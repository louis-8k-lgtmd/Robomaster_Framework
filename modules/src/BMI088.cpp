#include "BMI088.hpp"
#include "spi.h"
#include "tx_api.h"
#include <cmath>

namespace BMI088
{
    namespace
    {
        // 阻塞式 SPI 操作的 HAL 超时上限，单位为 HAL tick（通常为毫秒）。
        constexpr uint32_t SPI_TIMEOUT = 10U;

        /*
         * 将逻辑传感器选择转换为板级 GPIO 片选：
         * BMI088 内部的加速度计和陀螺仪是两个独立 SPI 从设备，共用 SPI1 的
         * SCK/MISO/MOSI，但必须分别拉低各自的 CS 才能访问对应芯片。
         * SPI1 的主机模式、时钟极性/相位、位宽和波特率由 CubeMX 在 spi.c 中配置。
         * CS 低有效；返回 false 表示调用者传入了不支持的传感器编号。
         */
        bool GetChipSelect(BMI088_SENSOR sensor, GPIO_TypeDef **port, uint16_t *pin)
        {
            switch (sensor)
            {
            case BMI088_CS_ACC:
                *port = GPIOA;
                *pin = GPIO_PIN_4;
                return true;
            case BMI088_CS_GYRO:
                *port = GPIOB;
                *pin = GPIO_PIN_0;
                return true;
            default:
                return false;
            }
        }

        /*
         * 将两个连续低地址优先的 SPI 字节解码为带符号 16 位二补码数值。
         * 先组成无符号 16 位模式，再显式减去 2^16 转成负值，避免依赖
         * 超出 int16_t 范围的无符号到有符号窄化结果。
         * data 必须至少指向两个可读字节。
         */
        int16_t ReadInt16LE(const uint8_t *data)
        {
            const uint16_t value = static_cast<uint16_t>(data[0]) |
                                   (static_cast<uint16_t>(data[1]) << 8U);
            return static_cast<int16_t>(value < 0x8000U ? value : static_cast<int32_t>(value) - 0x10000);
        }

        /*
         * 统一记录底层通信失败：
         * INIT_ERR 表示设备通信/初始化链路出错；具体数据错误标志指出受影响芯片。
         * 未识别的 sensor 无法确定是哪颗芯片，保守地将 ACC 与 GYRO 数据都标为无效。
         */
        void SetCommError(cBMI088 *imu, BMI088_SENSOR sensor)
        {
            imu->self_test.INIT_ERR = true;
            if (sensor == BMI088_CS_ACC)
                imu->self_test.ACC_DATA_ERR = true;
            else if (sensor == BMI088_CS_GYRO)
                imu->self_test.GYRO_DATA_ERR = true;
            else
            {
                imu->self_test.ACC_DATA_ERR = true;
                imu->self_test.GYRO_DATA_ERR = true;
            }
        }
    }

    /**
     * @brief 通过静止采样估算陀螺仪三轴零偏。
     *
     * 将 4000 个角速度读数分别求平均，作为之后 ReadGyroData() 的扣除量。
     * 标定期间传感器必须静止；ThreadX sleep 的 tick 数决定采样间隔和总时长。
     * 若任一轴平均角速度绝对值超过 0.1 rad/s，则认为采样期间可能移动，
     * 设置 CALIBRATE_ERR 并恢复头文件中的预标定偏置。
     *
     * 此流程只标定陀螺仪，不计算加速度计重力方向或加速度偏置。
     * 当前实现不会因单次 SPI 读取错误而中止整段采样；启动标定前应确认
     * 芯片初始化和陀螺仪数据状态正常，并在静止条件下执行。
     */
    void cBMI088::Calibrate()
    {
        // 采样量固定为 4000 个；累加变量以物理单位 rad/s 保存读数和。
        const int calib_samples = 4000; // 采样次数
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        gyro_data_t temp_gyro;

        // ReadGyroData 每次都会扣除 Gyro_offset，先清零才能得到未扣偏置的样本。
        Gyro_offset[0] = 0.0f;
        Gyro_offset[1] = 0.0f;
        Gyro_offset[2] = 0.0f;

        // 逐轴累加静止角速度；读数有效性由 ReadReg 更新 self_test 标志。
        for (int i = 0; i < calib_samples; i++)
        {
            ReadGyroData(&temp_gyro); // 这里读取的是原始值（因为Offset已清零）
            gyro_sum[0] += temp_gyro.x;
            gyro_sum[1] += temp_gyro.y;
            gyro_sum[2] += temp_gyro.z;
            
            // 间隔一个 ThreadX tick，避免连续重复读取同一输出数据；实际频率取决于系统 tick。
            tx_thread_sleep(1);
        }

        // 平均值即估算的静止零偏，后续角速度读取时从原始值中扣除。
        Gyro_offset[0] = gyro_sum[0] / calib_samples;
        Gyro_offset[1] = gyro_sum[1] / calib_samples;
        Gyro_offset[2] = gyro_sum[2] / calib_samples;
        
        // 静止时平均角速度应接近零；超阈值通常意味着标定时有运动或数据异常。
        if (fabs(Gyro_offset[0]) > 0.1f || fabs(Gyro_offset[1]) > 0.1f || fabs(Gyro_offset[2]) > 0.1f)
        {
            self_test.CALIBRATE_ERR = true;
            // 不采用可疑的新结果，回退至编译期提供的预标定偏置。
            Gyro_offset[0] = BMI088_GYRO_PRE_CALI_OFFSET_X; 
            Gyro_offset[1] = BMI088_GYRO_PRE_CALI_OFFSET_Y;
            Gyro_offset[2] = BMI088_GYRO_PRE_CALI_OFFSET_Z;
        }
        else
        {
            self_test.CALIBRATE_ERR = false;
        }
    }

    void cBMI088::TemperatureControl(float target_temp)
    {
        /*
         * BMI088 温控需要板级加热器、电源驱动及可用 PWM 通道共同支持。
         * 当前工程没有配置相应硬件输出，因此不执行 PID 或改变 GPIO/PWM，
         * 并保持 TEMP_CTRL_ERR，明确告知调用者此功能不可用。
         */
        (void)target_temp;
        self_test.TEMP_CTRL_ERR = true;
    }

    // 分别读取两个芯片的固定 ID，用于区分传感器未连接、片选错误或 SPI 链路异常。
    void cBMI088::VerifyAccChipID()
    {
        uint8_t id = 0;
        ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, &id, 1);
        // 芯片 ID 比较用于设置专属错误标志，再并入总初始化错误状态。
        self_test.ACC_CHIP_ID_ERR = id != ACC_CHIP_ID_VAL;
        self_test.INIT_ERR |= self_test.ACC_CHIP_ID_ERR;
    }

    void cBMI088::VerifyGyroChipID()
    {
        uint8_t id = 0;
        ReadReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, &id, 1);
        self_test.GYRO_CHIP_ID_ERR = id != GYRO_CHIP_ID_VAL;
        self_test.INIT_ERR |= self_test.GYRO_CHIP_ID_ERR;
    }

    void cBMI088::VerifyAccData()
    {
        // 先执行完整数据读取；若 SPI 已报告错误，不再用清零占位样本覆盖该错误原因。
        ReadAccData(&acc_data);
        if (!self_test.ACC_DATA_ERR)
        {
            self_test.ACC_DATA_ERR = !std::isfinite(acc_data.x) || !std::isfinite(acc_data.y) ||
                                     !std::isfinite(acc_data.z);
        }
    }

    void cBMI088::VerifyGyroData()
    {
        ReadGyroData(&gyro_data);
        if (!self_test.GYRO_DATA_ERR)
        {
            self_test.GYRO_DATA_ERR = !std::isfinite(gyro_data.x) || !std::isfinite(gyro_data.y) ||
                                      !std::isfinite(gyro_data.z);
        }
    }

    /*
     * 向指定 BMI088 芯片写一个寄存器地址及后续连续数据字节。
     * 传输采用阻塞 HAL SPI；CS 在地址和全部 payload 期间保持低电平，
     * 事务结束后恢复高电平。len=0 时只发送地址阶段。
     */
    void cBMI088::WriteReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        GPIO_TypeDef *port;
        uint16_t pin;
        // 零长度允许 data 为空；存在 payload 时必须给出有效缓冲区。
        if (!GetChipSelect(cs, &port, &pin) || (len != 0U && data == nullptr))
        {
            SetCommError(this, cs);
            return;
        }

        // BMI088 写事务的地址 bit7 清零；HAL 先发地址，随后才在同一 CS 下发 payload。
        uint8_t command = addr & BMI088_SPI_WRITE_CODE;
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &command, 1, SPI_TIMEOUT);
        // 地址发送失败时不继续发数据，避免把 payload 错当成寄存器地址。
        if (status == HAL_OK && len != 0U)
            status = HAL_SPI_Transmit(&hspi1, data, len, SPI_TIMEOUT);
        // 无论 HAL 返回成功或错误，都释放当前从设备片选。
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

        if (status != HAL_OK)
            SetCommError(this, cs);
    }

    /*
     * 从指定芯片的连续寄存器读取 len 个有效字节。
     * 读取前先清空目标缓冲区，若参数错误或 SPI 失败，调用方不会误用旧数据。
     */
    void cBMI088::ReadReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        GPIO_TypeDef *port;
        uint16_t pin;
        // 先清零输出；出错返回时数据保持为可识别的零值，并通过 self_test 报错。
        if (data != nullptr)
            for (uint16_t i = 0; i < len; ++i) data[i] = 0;
        if (!GetChipSelect(cs, &port, &pin) || data == nullptr || len == 0U)
        {
            SetCommError(this, cs);
            return;
        }

        /*
         * 两类芯片读时序不同：地址 bit7 置 1；加速度计在地址之后需额外丢弃
         * 一个 dummy byte（手册要求的读延迟），陀螺仪则直接开始有效数据读取。
         */
        uint8_t command = addr | BMI088_SPI_READ_CODE;
        uint8_t dummy = 0;
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &command, 1, SPI_TIMEOUT);
        // ACC 的首个接收字节无效，只负责产生额外 SCLK，因此单独读入 dummy 丢弃。
        if (status == HAL_OK && cs == BMI088_CS_ACC)
            status = HAL_SPI_Receive(&hspi1, &dummy, 1, SPI_TIMEOUT);
        if (status == HAL_OK)
            status = HAL_SPI_Receive(&hspi1, data, len, SPI_TIMEOUT);
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

        // 成功读取会清除对应数据错误标志；失败由 SetCommError 保持为错误状态。
        if (status != HAL_OK)
            SetCommError(this, cs);
        else if (cs == BMI088_CS_ACC)
            self_test.ACC_DATA_ERR = false;
        else
            self_test.GYRO_DATA_ERR = false;
    }

    void cBMI088::Config()
    {
        /*
         * 初始化分阶段进行：
         * 1) 清理/预置自检标志并等待上电稳定；
         * 2) 分别软复位 ACC 与 GYRO，等待复位完成；
         * 3) 校验两个独立芯片 ID，任一校验失败就停止，不继续写运行配置；
         * 4) 配置 ACC 上电、量程、ODR/带宽，再配置 GYRO 量程、带宽和正常模式。
         *
         * 各延时参数以 ThreadX tick 为单位，用于满足器件上电、复位和寄存器切换时序。
         */
        uint8_t value;
        self_test.INIT_ERR = false;
        self_test.ACC_CHIP_ID_ERR = true;
        self_test.GYRO_CHIP_ID_ERR = true;
        tx_thread_sleep(10);

        value = ACC_SOFTRESET_VAL;
        // 复位命令必须对两颗内部芯片分别发送，各自使用独立 CS。
        WriteReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &value, 1);
        value = GYRO_SOFTRESET_VAL;
        WriteReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &value, 1);
        tx_thread_sleep(100);

        VerifyAccChipID();
        VerifyGyroChipID();
        // 此处只在芯片 ID 阶段失败时立即终止；后续寄存器写错误通过 self_test 标志记录。
        if (self_test.INIT_ERR)
            return;

        value = ACC_PWR_CTRL_ON;
        // 先使能加速度计电源，再退出 suspend 并设置采样参数。
        WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &value, 1);
        tx_thread_sleep(150);
        value = ACC_PWR_CONF_ACT;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &value, 1);
        tx_thread_sleep(10);
        value = ACC_RANGE_6G;
        WriteReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &value, 1);
        tx_thread_sleep(5);
        /*
         * ACC_CONF 的组合值选择 ODR=800 Hz 与 normal 带宽；
         * ACC_RANGE 前面配置为 ±6 g，数据解析比例必须与此配置保持一致。
         */
        value = 0xAB;
        WriteReg(BMI088_CS_ACC, ACC_CONF_ADDR, &value, 1);
        tx_thread_sleep(5);

        value = GYRO_RANGE_2000_DEG_S;
        // 陀螺仪配置 ±2000°/s 满量程，原始计数在读取时换算为 rad/s。
        WriteReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &value, 1);
        tx_thread_sleep(5);
        value = GYRO_ODR_2000Hz_BANDWIDTH_230Hz;
        // 选择 2000 Hz 输出数据率及 230 Hz 带宽档位。
        WriteReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &value, 1);
        tx_thread_sleep(5);
        value = GYRO_LPM1_NOR;
        // 使陀螺仪处于正常工作模式，而非 suspend/deep-suspend。
        WriteReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &value, 1);
        tx_thread_sleep(5);
    }


    void cBMI088::ReadAccData(acc_data_t *data)
    {
        // 输出指针由调用方提供；无效时无法返回结构体数据，标记 ACC 数据错误。
        if (data == nullptr)
        {
            self_test.ACC_DATA_ERR = true;
            return;
        }

        uint8_t raw[ACC_XYZ_LEN];
        // X/Y/Z 各由低字节和高字节组成，ReadReg 按 ACC SPI 时序丢弃首个无效字节。
        ReadReg(BMI088_CS_ACC, ACC_X_LSB_ADDR, raw, sizeof(raw));
        if (self_test.ACC_DATA_ERR)
        {
            // 避免通信失败时向上层留下上一次的有效加速度样本。
            *data = {};
            return;
        }

        // 组合 little-endian 有符号原始计数，再按 ±6g 灵敏度换算为 m/s^2。
        data->x = ReadInt16LE(&raw[0]) * IMU_ACCEL_6G_SEN;
        data->y = ReadInt16LE(&raw[2]) * IMU_ACCEL_6G_SEN;
        data->z = ReadInt16LE(&raw[4]) * IMU_ACCEL_6G_SEN;
        data->temperature = 0.0f;
    }

    void cBMI088::ReadGyroData(gyro_data_t *data)
    {
        // 对外返回三轴角速度，单位为 rad/s。
        if (data == nullptr)
        {
            self_test.GYRO_DATA_ERR = true;
            return;
        }

        uint8_t raw[GYRO_XYZ_LEN];
        // 陀螺仪同样按 X/Y/Z 低字节优先连续读取，但没有加速度计的额外 dummy byte。
        ReadReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR, raw, sizeof(raw));
        if (self_test.GYRO_DATA_ERR)
        {
            *data = {};
            return;
        }

        // 将 ±2000°/s 原始计数乘以 rad/s/LSB 比例，再减去静止标定零偏。
        data->x = ReadInt16LE(&raw[0]) * IMU_GYRO_2000_SEN - Gyro_offset[0];
        data->y = ReadInt16LE(&raw[2]) * IMU_GYRO_2000_SEN - Gyro_offset[1];
        data->z = ReadInt16LE(&raw[4]) * IMU_GYRO_2000_SEN - Gyro_offset[2];
    }

    void cBMI088::ReadAccTemperature(float *temp)
    {
        // 温度来自加速度计 die，不代表板级环境温度或陀螺仪温度。
        if (temp == nullptr)
        {
            self_test.ACC_DATA_ERR = true;
            return;
        }

        uint8_t raw[TEMP_LEN];
        // 温度 MSB/LSB 是专用编码格式，不可按普通 16 位加速度计数直接解释。
        ReadReg(BMI088_CS_ACC, TEMP_MSB_ADDR, raw, sizeof(raw));
        if (self_test.ACC_DATA_ERR)
        {
            *temp = 0.0f;
            return;
        }

        // 11 位温度码由 MSB 的 8 位和 LSB 的高 3 位组成，LSB 低 5 位不使用。
        int16_t value = static_cast<int16_t>((static_cast<uint16_t>(raw[0]) << 3U) | (raw[1] >> 5U));
        // 符号扩展 11 位二补码：bit10 为符号位，负值需减去 2^11。
        if ((value & 0x0400) != 0)
            value -= 0x0800;
        if (value == -2048)
        {
            // 手册定义 -2048 为温度数据无效标记。
            self_test.ACC_DATA_ERR = true;
            *temp = 0.0f;
            return;
        }
        // 按数据手册温度传递函数换算为摄氏度：LSB 步进 0.125°C，基准偏移 23°C。
        *temp = value * TEMP_UNIT + TEMP_BIAS;
    }

}
