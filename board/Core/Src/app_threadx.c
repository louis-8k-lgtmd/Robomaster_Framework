/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_usart_bridge.h"
#include "bsp_can.hpp"
#include "imu_task.h"
#include "led.h"
#include "control_task.h"
#include "usart.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * UART 接收队列容量与单帧容量：
 * DMA 接收缓冲区大小与线程软件队列的单帧最大长度保持一致；
 * 队列最多暂存 USART_QUEUE_DEPTH 帧，过长数据或队列已满时计入丢弃计数。
 */
#define USART_FRAME_SIZE   256U
#define USART_QUEUE_DEPTH 4U
/*
 * ThreadX tick 为系统时间单位：
 * IMU 工作线程每 1/20 秒报告一次心跳；存活监控等待单个心跳的超时为 1/5 秒。
 * 实际毫秒值取决于 TX_TIMER_TICKS_PER_SECOND 的配置。
 */
#define WORKER_HEARTBEAT_TICKS (TX_TIMER_TICKS_PER_SECOND / 20U)
#define HEARTBEAT_TIMEOUT_TICKS (TX_TIMER_TICKS_PER_SECOND / 5U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
typedef struct
{
  UART_HandleTypeDef *huart;           /* 帧来源，用于将数据交给对应串口协议处理器。 */
  uint16_t size;                       /* DMA 本次事件中实际收到的有效字节数。 */
  uint8_t data[USART_FRAME_SIZE];      /* ISR 将 DMA 数据复制到这里，解除 DMA 缓冲区生命周期耦合。 */
} USART_Frame;

/*
 * ThreadX 对象采用静态存储，内存随应用全生命周期存在。
 * 线程创建函数传入 sizeof(stack)，因此栈容量参数的单位是字节；
 * USART/IMU/电机线程各有独立控制块和栈，避免共享栈导致任务互相破坏。
 */
static TX_THREAD usart_thread;
static ULONG usart_thread_stack[512];
static TX_SEMAPHORE usart_rx_ready;
static TX_THREAD imu_thread;
static ULONG imu_thread_stack[768];
static TX_SEMAPHORE imu_heartbeat;
static TX_SEMAPHORE imu_operation;
static TX_THREAD motor_thread;
static ULONG motor_thread_stack[512];
static TX_SEMAPHORE motor_heartbeat;
static TX_SEMAPHORE motor_operation;
static TX_THREAD alive_thread;
static ULONG alive_thread_stack[512];
/*
 * 各 ready 标志用于状态灯与工作线程门控：
 * startup_ready 表示 ThreadX 对象和全部线程已创建；
 * can_ready 表示 CAN BSP 启动成功；control_ready 表示电机注册成功；
 * imu_ready 表示 BMI088 初始化成功。它们不等同于每次周期操作都成功。
 */
static volatile uint8_t startup_ready;
static volatile uint8_t can_ready;
static volatile uint8_t imu_ready;
static volatile uint8_t control_ready;
volatile BMI088_Sample g_imu_sample;
/*
 * 供 IMU_GetLatest() 读取的最新完整样本。IMU 线程写入，其他线程通过临界区快照读取。
 * 启动消息使用静态存储期，因为 UART DMA 发送是异步操作，源缓冲区必须持续有效。
 */
static uint8_t usart1_startup_message[] = "USART DMA ready\r\n";
volatile HAL_StatusTypeDef usart1_startup_tx_status;
static USART_Frame usart_frames[USART_QUEUE_DEPTH];
/*
 * 单生产者/单消费者环形队列：
 * USART DMA 回调是生产者，推进 head 并增加 count；
 * USART 工作线程是消费者，推进 tail 并减少 count；
 * 访问共享队列元数据时由短临界区避免中断与线程同时改写。
 */
static volatile uint8_t usart_queue_head;
static volatile uint8_t usart_queue_tail;
static volatile uint8_t usart_queue_count;
volatile uint32_t usart_rx_dropped_frames;
volatile uint32_t usart1_rx_frames;
volatile uint32_t usart6_rx_frames;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static VOID USART_ThreadEntry(ULONG argument);
static VOID IMU_ThreadEntry(ULONG argument);
static VOID Motor_ThreadEntry(ULONG argument);
static VOID Alive_ThreadEntry(ULONG argument);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_Init */
  /*
   * 应用采用静态栈和静态对象，不从 ThreadX byte pool 动态分配内存。
   * 参数仍由 ThreadX 初始化入口传入，当前实现不需要使用它。
   */
  (void)byte_pool;
  /* 每次启动都从“未就绪”开始，避免沿用未初始化的全局状态。 */
  startup_ready = 0U;
  can_ready = 0U;
  imu_ready = 0U;
  control_ready = 0U;

  /*
   * 先启动 CAN 接收/发送能力，再注册电机实例。
   * 若 CAN 初始化失败，不调用电机注册；后续控制线程仍创建并发心跳，
   * 但因 control_ready 为零不会执行电机控制步进，Alive 灯可报告启动故障。
   */
  can_ready = CAN_Init() ? 1U : 0U;
  if (can_ready)
  {
    control_ready = ControlTask_Init();
  }

  /*
   * 创建计数信号量，初值 0 表示还没有待处理事件：
   * usart_rx_ready：DMA 回调每成功入队一帧 put 一次，USART 线程等待后取帧；
   * heartbeat：工作线程周期性 put，存活监控用超时 get 检查线程是否仍运行；
   * operation：仅本周期功能操作成功时 put，用于区别“线程存活”与“工作正常”。
   */
  ret = tx_semaphore_create(&usart_rx_ready, "USART RX ready", 0);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  ret = tx_semaphore_create(&imu_heartbeat, "IMU heartbeat", 0);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  ret = tx_semaphore_create(&imu_operation, "IMU operation", 0);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  ret = tx_semaphore_create(&motor_heartbeat, "Motor heartbeat", 0);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  ret = tx_semaphore_create(&motor_operation, "Motor operation", 0);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  /*
   * 创建线程时均立即 TX_AUTO_START。
   * ThreadX 优先级值越小优先级越高；Alive 监控优先级最低，UART/IMU/电机任务
   * 按各自实时性设定优先级。每次创建后立即检查状态，失败时向上返回错误码，
   * 不继续创建依赖不完整的后续任务。
   */
  ret = tx_thread_create(&alive_thread, "Alive monitor",
                         Alive_ThreadEntry, 0,
                         alive_thread_stack, sizeof(alive_thread_stack),
                         15, 15, TX_NO_TIME_SLICE, TX_AUTO_START);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  ret = tx_thread_create(&usart_thread, "USART communication",
                         USART_ThreadEntry, 0,
                         usart_thread_stack, sizeof(usart_thread_stack),
                         10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  ret = tx_thread_create(&imu_thread, "BMI088",
                         IMU_ThreadEntry, 0,
                         imu_thread_stack, sizeof(imu_thread_stack),
                         8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  ret = tx_thread_create(&motor_thread, "Motor control",
                         Motor_ThreadEntry, 0,
                         motor_thread_stack, sizeof(motor_thread_stack),
                         9, 9, TX_NO_TIME_SLICE, TX_AUTO_START);
  /*
   * motor_thread 是最后一个创建的线程；只有创建成功才设置 startup_ready。
   * 该标志表示调度对象部署完成，不表示 CAN/IMU/电机反馈等运行状态全部正常。
   */
  if (ret == TX_SUCCESS)
  {
    startup_ready = 1U;
  }
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  /*
   * 进入 ThreadX 内核并启动调度器。
   * App_ThreadX_Init 由 ThreadX 启动流程调用，创建应用对象后调度各线程；
   * 正常运行时该调用不会返回到 main()。
   */
  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
/*
 * UART 帧处理扩展点。__weak 允许其他应用文件提供同名强定义以替换默认实现；
 * 当前默认行为不解析协议、不回传数据，只分别累计 USART1/USART6 的收帧数量。
 * 该函数由 USART 工作线程调用，而非 DMA/ UART 中断上下文。
 */
__weak void USART_ProcessFrame(UART_HandleTypeDef *huart,
                               const uint8_t *data, uint16_t size)
{
  (void)data;
  (void)size;
  if (huart->Instance == USART1)
  {
    ++usart1_rx_frames;
  }
  else if (huart->Instance == USART6)
  {
    ++usart6_rx_frames;
  }
}

/*
 * BSP 的 Receive-to-Idle DMA 回调转入此队列入口。
 * 在中断上下文中只做长度检查、短数据复制、更新队列索引并唤醒线程；
 * 实际协议处理留给 USART 线程，以缩短 ISR 执行时间并避免在中断中阻塞。
 */
void USART_DMA_RxFrame(UART_HandleTypeDef *huart, const uint8_t *data,
                       uint16_t size)
{
  USART_Frame *frame;

  /* 无效帧或队列满时选择丢弃新帧，不覆盖消费者尚未取走的旧帧。 */
  if (size == 0U || size > USART_FRAME_SIZE || usart_queue_count == USART_QUEUE_DEPTH)
  {
    ++usart_rx_dropped_frames;
    return;
  }

  // head 指向下一个空槽；数据先完整复制，再发布新的 head/count。
  frame = &usart_frames[usart_queue_head];
  frame->huart = huart;
  frame->size = size;
  memcpy(frame->data, data, size);
  usart_queue_head = (uint8_t)((usart_queue_head + 1U) % USART_QUEUE_DEPTH);
  ++usart_queue_count;
  // 计数信号量唤醒等待中的消费者；忽略返回码是因为 ISR 只做通知且对象已预建。
  (void)tx_semaphore_put(&usart_rx_ready);
}

/*
 * 对外提供 IMU 最近一次已发布样本的线程安全快照。
 * 临界区只保护结构体复制，不包含 SPI 读取或计算，保持关中断时间尽量短。
 */
void IMU_GetLatest(BMI088_Sample *sample)
{
  UINT interrupt_state;
  if (sample == NULL)
  {
    return;
  }

  /* 结构体快照在临界区内复制，避免读取到线程更新一半的数据。 */
  interrupt_state = tx_interrupt_control(TX_INT_DISABLE);
  *sample = (BMI088_Sample)g_imu_sample;
  tx_interrupt_control(interrupt_state);
}

static VOID USART_ThreadEntry(ULONG argument)
{
  USART_Frame frame;
  UINT interrupt_state;
  (void)argument;

  /*
   * main() 已在进入 ThreadX 前初始化 DMA/UART 硬件。
   * 在调度器启动后才开启 Receive-to-Idle DMA，确保队列和信号量等 RTOS 对象已就绪。
   */
  USART_DMA_Start();
  // 上电提示仅发一次；状态保存在全局变量供调试查看，DMA 缓冲区为静态数组。
  usart1_startup_tx_status = USART_DMA_Transmit(
      &huart1, usart1_startup_message, sizeof(usart1_startup_message) - 1U);

  for (;;)
  {
    /*
     * 阻塞等待 RX 信号量，队列为空时不轮询。
     * 被唤醒后在短临界区取出一帧并释放队列槽位，随后恢复中断再运行协议处理。
     */
    if (tx_semaphore_get(&usart_rx_ready, TX_WAIT_FOREVER) != TX_SUCCESS)
    {
      continue;
    }

    // 保护 count/tail 和帧拷贝，防止 DMA ISR 在读取队列状态时并发写入。
    interrupt_state = tx_interrupt_control(TX_INT_DISABLE);
    if (usart_queue_count == 0U)
    {
      tx_interrupt_control(interrupt_state);
      continue;
    }
    // 将槽位内容复制到线程栈局部变量，释放槽位后再处理，生产者可立即重用该槽。
    frame = usart_frames[usart_queue_tail];
    usart_queue_tail = (uint8_t)((usart_queue_tail + 1U) % USART_QUEUE_DEPTH);
    --usart_queue_count;
    tx_interrupt_control(interrupt_state);

    // 可能耗时的协议解析位于中断使能状态下的线程上下文。
    USART_ProcessFrame(frame.huart, frame.data, frame.size);
  }
}

static VOID IMU_ThreadEntry(ULONG argument)
{
  BMI088_Sample sample = {0};
  uint8_t operation_ok;
  (void)argument;

  for (;;)
  {
    /*
     * 初始化失败时在后续周期重试；初始化成功后每轮读取一次样本。
     * 该线程是 BMI088 驱动的唯一调用者，避免并发 SPI 访问和标志竞争。
     */
    if (!imu_ready)
    {
      imu_ready = BMI088_TaskInit();
    }
    else
    {
      operation_ok = BMI088_TaskRead(&sample);
      /*
       * BMI088_TaskRead 先生成局部 sample，再在临界区中整体发布全局快照。
       * operation 信号仅代表本次加速度、角速度、温度读取全部有效；
       * 失败时仍发布其有效标志/清零数据，供上层观察本轮结果。
       */
      UINT interrupt_state = tx_interrupt_control(TX_INT_DISABLE);
      g_imu_sample = sample;
      tx_interrupt_control(interrupt_state);
      if (operation_ok)
      {
        (void)tx_semaphore_put(&imu_operation);
      }
    }

    /*
     * 无论初始化或采样成功与否都发送 heartbeat：
     * Alive 线程可据此识别“线程还在跑，但传感器功能失败”的情况。
     */
    (void)tx_semaphore_put(&imu_heartbeat);
    tx_thread_sleep(WORKER_HEARTBEAT_TICKS);
  }
}

extern uint8_t MotorControlTaskStep(void);

static VOID Motor_ThreadEntry(ULONG argument)
{
  (void)argument;
  for (;;)
  {
    /*
     * 约 100 Hz 电机循环。control_ready 为真时执行一次控制计算和 CAN 发送；
     * 未就绪时跳过控制步进，但仍发送 heartbeat，便于将调度存活与控制可用性区分。
     */
    uint8_t operation_ok = 0U;
    if (control_ready)
    {
      operation_ok = MotorControlTaskStep();
    }
    if (operation_ok)
    {
      (void)tx_semaphore_put(&motor_operation);
    }
    // operation 只表示本次控制步进报告成功；heartbeat 每周期无条件递增。
    (void)tx_semaphore_put(&motor_heartbeat);
    tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 100U);
  }
}

static UINT WaitForHeartbeat(TX_SEMAPHORE *heartbeat,
                             TX_SEMAPHORE *operation,
                             UINT *operation_ok)
{
  /*
   * 检查一个监控窗口：
   * 1) 最多阻塞 HEARTBEAT_TIMEOUT_TICKS 等待工作线程的一个心跳；
   * 2) 若已收到，清空其余积压计数，避免工作线程短暂领先时旧心跳掩盖停滞；
   * 3) 非阻塞排空 operation 信号量，只要窗口内至少有一次成功就置 operation_ok。
   * 返回值仅描述 heartbeat 是否在超时内到达，operation_ok 单独描述功能成功。
   */
  UINT status = tx_semaphore_get(heartbeat, HEARTBEAT_TIMEOUT_TICKS);
  if (status == TX_SUCCESS)
  {
    while (tx_semaphore_get(heartbeat, TX_NO_WAIT) == TX_SUCCESS)
    {
    }
  }
  *operation_ok = 0U;
  while (tx_semaphore_get(operation, TX_NO_WAIT) == TX_SUCCESS)
  {
    *operation_ok = 1U;
  }
  return status;
}

static VOID Alive_ThreadEntry(ULONG argument)
{
  UINT imu_alive;
  UINT motor_alive;
  UINT imu_ok;
  UINT motor_ok;
  (void)argument;

  for (;;)
  {
    /*
     * LED 状态由系统启动、外设就绪、线程心跳和功能成功共同决定：
     * 红灯：任一关键初始化失败，或 IMU/电机线程无心跳，或本监控窗口功能未成功；
     * 绿灯：电机控制线程存活且窗口内至少一次 CAN 控制步进成功；
     * 蓝灯：IMU 初始化成功、线程存活且窗口内至少一次完整样本读取成功。
     * RGB 各通道分别控制，因此故障红灯可与仍正常工作的绿/蓝灯同时点亮。
     */
    imu_alive = WaitForHeartbeat(&imu_heartbeat, &imu_operation, &imu_ok) == TX_SUCCESS;
    motor_alive = WaitForHeartbeat(&motor_heartbeat, &motor_operation, &motor_ok) == TX_SUCCESS;
    LED_SetRGB(!startup_ready || !can_ready || !control_ready ||
                   !imu_alive || !motor_alive || !imu_ready || !imu_ok || !motor_ok,
               motor_alive && motor_ok,
               imu_alive && imu_ready && imu_ok);
  }
}
/* USER CODE END 1 */
