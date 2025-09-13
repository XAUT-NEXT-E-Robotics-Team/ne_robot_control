#include "Vision_Conection.h"
#include "usbd_cdc_if.h"
/**
 * @author cherishQA
 * @version 0.1
 * @date 2025.7.25
 * 
 */

VisionState visionState;
VisionConnect vision1;
/**
 * @brief 视觉数据初始化
 * @param conect
 * @param OurColor
 */
 void Vision_ConectINIT( VisionConnect *conect , uint8_t OurColor )
 { 
   conect->RXData.VisionRxData.PitchAngleTarget = 0.0f ; 
   conect->RXData.VisionRxData.YawAngleTarget   = 0.0f ;   
   conect->RXData.VisionRxData.fireControl = HOLD_FIRE ;
   conect->RXData.VisionRxData.header = 0 ;
   conect->TXData.PitchAngle = 0.0F ;
   conect->TXData.YawAngle = 0.0F ;
   conect->TXData.OurColor = OurColor ;
   conect->TXData.ShootSpeed = 0.0f ;
 } 


/**
 * @brief 更新视觉数据
 * @param  connect:
 * @param  OurColor:
 * @param  yawAngle:
 * @param  pitchAngle:
 * @param  shootSpeed:
 */
void VisionConnectUpdateTX(VisionConnect *connect, uint8_t OurColor, float yawAngle,
                           float pitchAngle, float shootSpeed) {
  connect->TXData.OurColor = OurColor;

  connect->TXData.PitchAngle = -pitchAngle;	// 反转pitch轴
  connect->TXData.YawAngle = yawAngle + 180.0f;	// 将yawAngle的范围由-180~180改为0~360
  connect->TXData.ShootSpeed = shootSpeed;
}

/**
 * @brief  向视觉上位机发送数据
 * @param  connnect:
 */
void VisionConnectSend(VisionConnect *connnect) {

  connnect->TXData.packHead = 0x00a5;  // 包头
  // 我方颜色
   if (connnect->TXData.OurColor == RED) {
     connnect->TXData.OurColor = RED;
   }
   else if (connnect->TXData.OurColor == BLUE) {
     connnect->TXData.OurColor = BLUE;
   }
	
  // 发送
  CDC_Transmit_FS((uint8_t *)&(connnect->TXData), sizeof(connnect->TXData));
}

/** 
 * @brief 解包state
 * 
 * */ 
void VisionGetState(VisionConnect* connect, VisionState* visionState)
{
	uint8_t state = connect->RXData.VisionRxData.state;
	visionState->aoto_aim_state = state & 0x01;
	visionState->our_color = (state & 0x02) >> 1;
	visionState->tracking = (state & 0x04) >> 2;
	visionState->aim_type = (state & 0x18) >> 3;
}
