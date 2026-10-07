#include "imu_task.h"
#include "BMI088.hpp"

namespace
{
/*
 * IMU 模块拥有唯一 BMI088 驱动实例和初始化状态。
 * 驱动通过阻塞 SPI 访问设备，约定仅由 IMU 工作线程调用，避免多个调用者
 * 同时切换同一组 CS/SPI 总线；该实例具有静态存储期，可在多次任务调用间保留偏置。
 */
BMI088::cBMI088 imu;
bool imu_ready;
}

extern "C" uint8_t BMI088_TaskInit(void)
{
    /*
     * 初始化 BMI088 的两颗内部芯片。Config 内执行复位、芯片 ID 校验及运行参数写入；
     * 只有总初始化标志和两个芯片 ID 标志都通过，才把设备标记为可读取。
     */
    imu.Config();
    imu_ready = !imu.self_test.INIT_ERR &&
                !imu.self_test.ACC_CHIP_ID_ERR &&
                !imu.self_test.GYRO_CHIP_ID_ERR;
    return imu_ready ? 1U : 0U;
}

extern "C" uint8_t BMI088_TaskRead(BMI088_Sample *sample)
{
    /*
     * 读取一个完整采样快照。sample 是调用者提供的输出结构；
     * 未初始化时不访问 SPI，也不承诺改写输出结构。
     */
    if (sample == nullptr || !imu_ready)
        return 0U;

    // 先在局部对象中准备结果，避免数据读取过程向上层暴露未填写的分量。
    acc_data_t acc{};
    gyro_data_t gyro{};
    float temperature = 0.0f;
    /*
     * 三次读取分开执行且每次之后立即记录对应有效标志：
     * ACC 读与温度读共享 ACC_DATA_ERR，因此必须在温度读取改写该标志前保存
     * acc_valid；GYRO 使用独立的 GYRO_DATA_ERR。
     */
    imu.ReadAccData(&acc);
    sample->acc_valid = imu.self_test.ACC_DATA_ERR ? 0U : 1U;
    imu.ReadGyroData(&gyro);
    sample->gyro_valid = imu.self_test.GYRO_DATA_ERR ? 0U : 1U;
    imu.ReadAccTemperature(&temperature);
    sample->temperature_valid = imu.self_test.ACC_DATA_ERR ? 0U : 1U;

    // 即使某项无效也复制驱动给出的结果（失败时驱动会将输出清零），有效性由标志判定。
    sample->accel[0] = acc.x;
    sample->accel[1] = acc.y;
    sample->accel[2] = acc.z;
    sample->gyro[0] = gyro.x;
    sample->gyro[1] = gyro.y;
    sample->gyro[2] = gyro.z;
    sample->temperature = temperature;
    // 聚合返回值要求三个分项全部有效；调用方也可单独使用各 valid 标志降级处理。
    return sample->acc_valid && sample->gyro_valid && sample->temperature_valid;
}
