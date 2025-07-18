#include "referee.h"

/* 外部变量声明 */
extern UART_HandleTypeDef huart6;        // UART6句柄，用于与裁判系统通信
extern DMA_HandleTypeDef hdma_usart6_rx; // UART6接收DMA句柄
extern DMA_HandleTypeDef hdma_usart6_tx; // UART6发送DMA句柄

/* 全局变量定义 */
uint8_t usart_buf[2][USART_RX_BUF_LENGHT];        // 双缓冲区，用于DMA接收数据
fifo_s_t referee_fifo;                             // 裁判系统数据FIFO缓冲区
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH]; // FIFO缓冲区数组
unpack_data_t referee_unpack_obj;                  // 数据解包对象
frame_header_struct_t referee_receive_header;      // 接收数据帧头
frame_header_struct_t referee_send_header;         // 发送数据帧头

/* 裁判系统数据结构体定义 - 根据通信协议v1.7定义 */
game_state_t game_state;                              // 0x0001 比赛状态数据
game_result_t game_result;                            // 0x0002 比赛结果数据
game_robot_HP_t game_robot_HP;                        // 0x0003 机器人血量数据
event_data_t event_data;                              // 0x0101 场地事件数据
referee_warning_t referee_warning;                    // 0x0104 裁判警告信息
dart_remaining_time_t dart_remaining_time;            // 0x0105 飞镖发射口倒计时
robot_state_t robot_state;                            // 0x0201 机器人状态数据
power_heat_data_t power_heat_data;                    // 0x0202 实时功率热量数据
game_robot_pos_t game_robot_pos;                      // 0x0203 机器人位置数据
robot_buff_t robot_buff;                              // 0x0204 机器人增益数据
hurt_data_t hurt_data;                                // 0x0206 伤害状态数据
shoot_data_t shoot_data;                              // 0x0207 实时射击信息
projectile_allowance_t projectile_allowance;          // 0x0208 弹丸允许发射数据
rfid_status_t rfid_status;                            // 0x0209 RFID状态数据
dart_client_cmd_t dart_client_cmd;                    // 0x020A 飞镖选手端指令数据
ground_robot_position_t ground_robot_position;        // 0x020B 地面机器人位置数据
radar_mark_data_t radar_mark_data;                    // 0x020C 雷达标记进度数据
robot_interaction_data_t robot_interaction_data;      // 0x0301 机器人间交互数据
referee_remote_control_t referee_remote_control;      // 0x0304 裁判系统遥控器数据
robot_custom_data_t robot_custom_data;			      // 0x0309 自定义控制器数据


/**
 * @brief  初始化裁判系统数据结构体
 * @param  None
 * @retval None
 * @note   将所有裁判系统相关的数据结构体清零初始化
 */
void init_referee_struct_data(void) {
  // 初始化帧头结构体
  memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
  memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

  // 初始化比赛基本信息数据结构体
  memset(&game_state, 0, sizeof(game_state_t));           // 比赛状态数据
  memset(&game_result, 0, sizeof(game_result_t));         // 比赛结果数据
  memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));     // 机器人血量数据

  // 初始化场地事件相关数据结构体
  memset(&event_data, 0, sizeof(event_data_t));           // 场地事件数据
  memset(&referee_warning, 0, sizeof(referee_warning_t)); // 裁判警告信息

  // 初始化机器人状态相关数据结构体
  memset(&dart_remaining_time, 0, sizeof(dart_remaining_time_t));     // 飞镖发射口倒计时
  memset(&robot_state, 0, sizeof(robot_state_t));                     // 机器人状态数据
  memset(&power_heat_data, 0, sizeof(power_heat_data_t));             // 实时功率热量数据
  memset(&game_robot_pos, 0, sizeof(game_robot_pos_t));               // 机器人位置数据
  memset(&robot_buff, 0, sizeof(robot_buff_t));                       // 机器人增益数据
  memset(&hurt_data, 0, sizeof(hurt_data_t));                         // 伤害状态数据
  memset(&shoot_data, 0, sizeof(shoot_data_t));                       // 实时射击信息
  memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));   // 弹丸允许发射数据
  memset(&rfid_status, 0, sizeof(rfid_status_t));                     // RFID状态数据
  memset(&dart_client_cmd, 0, sizeof(dart_client_cmd_t));             // 飞镖选手端指令数据
  memset(&ground_robot_position, 0, sizeof(ground_robot_position_t)); // 地面机器人位置数据
  memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));             // 雷达标记进度数据
  memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t)); // 机器人间交互数据
  memset(&referee_remote_control, 0, sizeof(referee_remote_control_t)); // 裁判系统遥控器数据
}

/**
 * @brief  裁判系统数据解析函数
 * @param  frame: 接收到的数据帧指针
 * @retval None
 * @note   根据命令ID解析不同类型的裁判系统数据包
 */
void referee_data_solve(uint8_t *frame) {
  uint16_t cmd_id = 0;    // 命令ID
  uint8_t index = 0;      // 数据索引

  // 复制帧头信息
  memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));
  index += sizeof(frame_header_struct_t);

  // 提取命令ID
  memcpy(&cmd_id, frame + index, sizeof(uint16_t));
  index += sizeof(uint16_t);

  // 根据命令ID解析对应的数据包
  switch (cmd_id) {
    case GAME_STATE_CMD_ID: {  // 0x0001 比赛状态数据
      memcpy(&game_state, frame + index, sizeof(game_state_t));
    } break;
    case GAME_RESULT_CMD_ID: {  // 0x0002 比赛结果数据
      memcpy(&game_result, frame + index, sizeof(game_result_t));
    } break;
    case GAME_ROBOT_HP_CMD_ID: {  // 0x0003 机器人血量数据
      memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
    } break;
    case EVENTS_CMD_ID: {  // 0x0101 场地事件数据
      memcpy(&event_data, frame + index, sizeof(event_data_t));
    } break;
    case REFEREE_WARNING_CMD_ID: {  // 0x0104 裁判警告信息
      memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
    } break;
    case DART_REMAINING_TIME_CMD_ID: {  // 0x0105 飞镖发射口倒计时
      memcpy(&dart_remaining_time, frame + index, sizeof(dart_remaining_time_t));
    } break;
    case ROBOT_STATE_CMD_ID: {  // 0x0201 机器人状态数据
      memcpy(&robot_state, frame + index, sizeof(robot_state_t));
    } break;
    case POWER_HEAT_DATA_CMD_ID: {  // 0x0202 实时功率热量数据
      memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
    } break;
    case GAME_ROBOT_POS_CMD_ID: {  // 0x0203 机器人位置数据
      memcpy(&game_robot_pos, frame + index, sizeof(game_robot_pos_t));
    } break;
    case ROBOT_BUFF_CMD_ID: {  // 0x0204 机器人增益数据
      memcpy(&robot_buff, frame + index, sizeof(robot_buff_t));
    } break;
    case HURT_DATA_CMD_ID: {  // 0x0206 伤害状态数据
      memcpy(&hurt_data, frame + index, sizeof(hurt_data_t));
    } break;
    case SHOOT_DATA_CMD_ID: {  // 0x0207 实时射击信息
      memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
    } break;
    case PROJECTILE_ALLOWANCE_CMD_ID: {  // 0x0208 弹丸允许发射数据
      memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
    } break;
    case RFID_STATUS_CMD_ID: {  // 0x0209 RFID状态数据
      memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
    } break;
    case DART_CLIENT_CMD_CMD_ID: {  // 0x020A 飞镖选手端指令数据
      memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd_t));
    } break;
    case GROUND_ROBOT_POSITION_CMD_ID: {  // 0x020B 地面机器人位置数据
      memcpy(&ground_robot_position, frame + index, sizeof(ground_robot_position_t));
    } break;
    case RADAR_MARK_DATA_CMD_ID: {  // 0x020C 雷达标记进度数据
      memcpy(&radar_mark_data, frame + index, sizeof(radar_mark_data_t));
    } break;
    case ROBOT_INTERACTION_DATA_CMD_ID: {  // 0x0301 机器人间交互数据
      memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
    } break;
    case REFEREE_REMOTE_CONTROL_CMD_ID: {  // 0x0304 裁判系统遥控器数据
      memcpy(&referee_remote_control, frame + index, sizeof(referee_remote_control_t));
    } break;
    default: {  // 未知命令ID，不做处理
      break;
    }
  }
}

/**
 * @brief  裁判系统初始化函数
 * @param  None
 * @retval None
 * @note   初始化裁判系统数据结构体、FIFO缓冲区和串口DMA
 */
void refereeINIT() {
  // 初始化裁判系统数据结构体
  init_referee_struct_data();
  
  // 初始化用于存储裁判系统接收数据的FIFO缓冲区
  fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
  
  // 初始化UART6接收的DMA配置，使用双缓冲模式
  referee_usart_init(&huart6, &hdma_usart6_rx, &hdma_usart6_tx, usart_buf[0], usart_buf[1],
                     USART_RX_BUF_LENGHT);
  // 裁判系统初始化完成，准备接收数据
}

/**
 * @brief  裁判系统FIFO数据解包函数
 * @param  None
 * @retval None
 * @note   每10ms调用一次，从FIFO中解包裁判系统数据帧
 *         采用状态机方式逐字节解析数据包，确保数据完整性
 */
void referee_unpack_fifo_data(void) {
  uint8_t byte = 0;                        // 当前读取的字节
  uint8_t sof = HEADER_SOF;               // 帧起始标识符 0xA5
  unpack_data_t *p_obj = &referee_unpack_obj;  // 解包对象指针

  // 循环处理FIFO中的所有数据
  while (fifo_s_used(&referee_fifo)) {
    byte = fifo_s_get(&referee_fifo);  // 从FIFO中取出一个字节
    switch (p_obj->unpack_step) {
        /********* 以下是对串口接收的一帧数据的帧头frame_header进行校验 *********/
      case STEP_HEADER_SOF: {  // 步骤1：检测帧起始标识符
        if (byte == sof) {  // 找到帧起始符0xA5
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else {  // 不是起始符，重新开始
          p_obj->index = 0;
        }
      } break;

      case STEP_LENGTH_LOW: {  // 步骤2：接收数据长度低字节
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      } break;

      case STEP_LENGTH_HIGH: {  // 步骤3：接收数据长度高字节
        p_obj->data_len |= (byte << 8);  // 组合成16位数据长度
        p_obj->protocol_packet[p_obj->index++] = byte;

        // 检查数据长度是否合理
        if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else {  // 数据长度异常，重新开始
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      } break;

      case STEP_FRAME_SEQ: {  // 步骤4：接收包序号
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      } break;

      case STEP_HEADER_CRC8: {  // 步骤5：接收帧头CRC8校验码
        p_obj->protocol_packet[p_obj->index++] = byte;
        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE) {
          // 验证帧头CRC8校验和
          if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
            p_obj->unpack_step = STEP_DATA_CRC16;  // 帧头校验成功，继续接收数据
          }
          else {  // 帧头校验失败，重新开始
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      } break;
        /********* 以上是对串口接收的一帧数据的帧头frame_header进行校验 *********/
      case STEP_DATA_CRC16: {  // 步骤6：接收数据内容和CRC16校验码
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
          p_obj->unpack_step = STEP_HEADER_SOF;  // 重置状态机
          p_obj->index = 0;

          // 验证整帧数据的CRC16校验和
          if (verify_CRC16_check_sum(p_obj->protocol_packet,
                                     REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
            referee_data_solve(p_obj->protocol_packet);  // 校验成功，解析数据包
          }
        }
      } break;

      default: {  // 异常状态，重新开始
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      } break;
    }
  }
}

/**
 * @brief  裁判系统串口空闲中断处理函数
 * @param  None
 * @retval None
 * @note   在串口接收中断中调用，处理双缓冲DMA接收的数据
 *         当检测到串口空闲时，将接收到的数据放入FIFO缓冲区
 */
void refereeReceiveHandler(void) {
  static volatile uint8_t res;
  
  // 检查是否产生串口空闲中断
  if (USART6->SR & UART_FLAG_IDLE) {
    __HAL_UART_CLEAR_PEFLAG(&huart6);  // 清除空闲中断标志

    static uint16_t this_time_rx_len = 0;  // 本次接收数据长度

    // 检查当前使用的内存缓冲区
    if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT)
        == RESET) { /* 当前使用的是内存缓冲区0 */

      // 关闭DMA，CPU处理当前缓冲区数据
      __HAL_DMA_DISABLE(huart6.hdmarx);

      // 获取接收数据长度 = 设定长度 - 剩余长度
      this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);

      // CPU重新设置DMA计数器
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);

      // CPU处理完缓冲区0数据后，切换DMA到内存缓冲区1
      huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;

      // 重新启用DMA
      __HAL_DMA_ENABLE(huart6.hdmarx);

      // CPU将内存缓冲区0的数据存入FIFO
      fifo_s_puts(&referee_fifo, (char *)usart_buf[0], this_time_rx_len);
      //  detect_hook(REFEREE_TOE); // 可选：离线检测钩子函数
    }
    else {
      /* 当前使用的是内存缓冲区1 */
      // 处理过程与缓冲区0相同
      __HAL_DMA_DISABLE(huart6.hdmarx);
      this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
      // 切换DMA到内存缓冲区0
      huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
      __HAL_DMA_ENABLE(huart6.hdmarx);
      // CPU将内存缓冲区1的数据存入FIFO
      fifo_s_puts(&referee_fifo, (char *)usart_buf[1], this_time_rx_len);
      //  detect_hook(REFEREE_TOE); // 可选：离线检测钩子函数
    }
  }
}
