#include "bsp_usart.hpp"
#include "bsp_usart_bridge.h"
#include "usart.h"

/*
 * 两路 Receive-to-Idle DMA 各自使用独立的静态接收缓冲区。
 * HAL/DMA 在接收期间持续写这些缓冲区；收到空闲事件后，回调把有效字节交给
 * ThreadX 队列接口，再复用原缓冲区启动下一轮 DMA。缓冲区不能是回调栈上的局部变量。
 */
static uint8_t uart1_rx_buffer[256];
static uint8_t uart6_rx_buffer[256];

/*
 * BSP 汇总初始化入口：分别启动 USART1 和 USART6 的 Receive-to-Idle DMA。
 * 此函数从 ThreadX USART 工作线程调用，而不是在 main 初始化阶段调用，
 * 这样接收回调触发时线程对象和接收队列已创建，回调可以安全提交完整帧。
 */
void USART_Init(void)
{
  USART1_Init();
  USART6_Init();
}

/*
 * C ABI 桥接函数，供 C 编写的 ThreadX 应用代码使用。
 * 只负责把 DMA 模式请求转交给统一发送接口；它不会复制发送数据，
 * 调用方应保证 data 在 HAL 返回发送完成之前一直有效。
 */
extern "C" void USART_DMA_Start(void)
{
  USART_Init();
}

extern "C" HAL_StatusTypeDef USART_DMA_Transmit(UART_HandleTypeDef *huart,
                                                 uint8_t *data, uint16_t size)
{
  return USART_Transmit(huart, data, size, USART_MODE_DMA);
}

/*
 * 启动 USART6 的 DMA 空闲接收。
 * Receive-to-Idle 让 HAL 在检测到线路空闲或缓冲区接收完成等事件时报告
 * 本次实际接收长度；因此应用可按变长帧处理，不必等满整个 256 字节缓冲区。
 */
void USART6_Init()
{
  // DMA 目标缓冲区为静态存储，长度以字节计；由 USART6 RX DMA 写入。
  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart6,
        uart6_rx_buffer,
        sizeof(uart6_rx_buffer)
    );
  if (status != HAL_OK)
  {
      Error_Handler();
  }
  // 禁止半传输中断，只在 Receive-to-Idle/传输完成事件处理一次本轮接收。
  __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
}

/* 与 USART6_Init 相同的配置流程，使用 USART1 专属句柄和接收缓冲区。 */
void USART1_Init()
{
  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart1,
        uart1_rx_buffer,
        sizeof(uart1_rx_buffer)
    );
  if (status != HAL_OK)
  {
      Error_Handler();
  }
  // 保持接收事件粒度一致，避免半缓冲区时额外触发回调。
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/*
 * HAL 的 Receive-to-Idle 接收事件回调。
 *
 * 调用时序：HAL 检测到一段数据结束 -> 本回调判断对应串口 -> 将有效字节交给
 * ThreadX 桥接回调复制入队 -> 重新将同一 DMA 缓冲区交给 HAL 接收下一段数据。
 * USART_DMA_RxFrame 应快速返回；协议解析在 USART 线程中执行，不在中断上下文执行。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  uint8_t *rx_buffer;
  uint16_t rx_buffer_size;

  // 根据 HAL 传回的 UART 句柄找到与之配对的静态 DMA 缓冲区和容量。
  if (huart == &huart1)
  {
    rx_buffer = uart1_rx_buffer;
    rx_buffer_size = sizeof(uart1_rx_buffer);
  }
  else if (huart == &huart6)
  {
    rx_buffer = uart6_rx_buffer;
    rx_buffer_size = sizeof(uart6_rx_buffer);
  }
  else
  {
    // 其他 UART 不属于本 BSP 的两路 DMA 接收，不读取或重启其 DMA。
    return;
  }

  /*
   * size 是本次接收的有效字节数，不是底层 DMA 缓冲区容量。
   * 先交给 ThreadX 队列接口：该接口会在返回前把数据复制到自己的帧槽位，
   * 所以下方重新启动 DMA 覆盖 rx_buffer 时，已入队的数据仍保持不变。
   */
  if (size > 0U && size <= rx_buffer_size)
  {
    USART_DMA_RxFrame(huart, rx_buffer, size);
  }

  // Receive-to-Idle DMA 在本次事件后停止/完成，必须重新提交缓冲区才能继续接收。
  if (HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, rx_buffer_size) != HAL_OK)
  {
    Error_Handler();
  }
  // 每次重启后都关闭半传输事件，维持“成段数据/空闲事件”回调策略。
  __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/*
 * 通用 UART 发送封装。
 * BLOCK 会一直等待传输完成；IT 和 DMA 会立即返回，实际完成发生在后续中断中。
 * 异步模式下 pData 的生命周期由调用者负责，不能在发送完成前释放或改写。
 */
HAL_StatusTypeDef USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
  // 长度为零没有可发送内容；空句柄或空缓冲区也不能交给 HAL。
  if (huart == nullptr || pData == nullptr || Size == 0U)
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status;
  // 按调用者选择的执行模型转发；这里不等待异步模式，也不静默改变模式。
  switch (mode)
  {
    case USART_MODE_BLOCK:
      // 阻塞直到 Size 字节发完或 HAL 返回错误；HAL_MAX_DELAY 表示不设软件超时。
      status = HAL_UART_Transmit(huart, pData, Size, HAL_MAX_DELAY);
      break;
    case USART_MODE_IT:
      // 启动中断驱动发送后立即返回，数据由 UART 中断逐步发送。
      status = HAL_UART_Transmit_IT(huart, pData, Size);
      break;
    case USART_MODE_DMA:
      // 启动内存到 UART 的 DMA 发送后立即返回，源缓冲区须继续有效。
      status = HAL_UART_Transmit_DMA(huart, pData, Size);
      break;
    default:
      // 未识别模式不尝试任何 HAL 操作。
      return HAL_ERROR;
  }

  // 返回 HAL 状态供调用方判断请求是否成功启动/完成（阻塞模式）。
  return status;
}

/*
 * 通用 UART 接收封装，按指定模式读取 Size 字节。
 * 与专用 Receive-to-Idle DMA 不同，本接口使用固定长度接收：
 * 若通信协议帧长度可变，应使用上面的 Receive-to-Idle 初始化和事件回调。
 */
void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
  // 接收数据必须有目标内存，且请求长度至少为一个字节。
  if (huart == nullptr || pData == nullptr || Size == 0U)
  {
    return;
  }

  /*
   * HAL UART 同一时刻只允许一个接收请求。
   * 若旧请求仍在活动，先尝试中止；中止失败则不覆盖旧请求或继续启动新请求。
   */
  if (huart->RxState != HAL_UART_STATE_READY && HAL_UART_AbortReceive(huart) != HAL_OK)
  {
    return;
  }

  HAL_StatusTypeDef status;
  // 选择阻塞、接收中断或 DMA 固定长度接收。
  switch (mode)
  {
    case USART_MODE_BLOCK:
      // 直到收到 Size 字节或 HAL 报错才返回。
      status = HAL_UART_Receive(huart, pData, Size, HAL_MAX_DELAY);
      break;
    case USART_MODE_IT:
      // 异步启动；完成后由 HAL 接收完成回调通知应用。
      status = HAL_UART_Receive_IT(huart, pData, Size);
      break;
    case USART_MODE_DMA:
      // 异步 DMA 写入 pData；接收完成前不能释放或复用该缓冲区。
      status = HAL_UART_Receive_DMA(huart, pData, Size);
      break;
    default:
      // 无效模式没有对应的接收操作。
      return;
  }

  // 仅 HAL_ERROR 进入统一错误处理；HAL_BUSY 等非错误状态保持原样返回给调用方。
  if (status == HAL_ERROR)
  {
    Error_Handler();
  }
}
