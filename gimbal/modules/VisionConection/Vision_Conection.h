#ifndef Vision_Conection_H
#define Vision_Conection_H

#include "main.h"

#define VISION_RECV_SIZE 18u
#define VISION_SEND_SIZE 36U


enum FireFlag {
  HOLD_FIRE = 0 , //开摩擦轮
  OPEN_FIRE = 1 , //拨弹
} ;

enum our_color{
  BLUE = 0 ,
  RED = 1 , 
};

#pragma pack(1)  //按1字对齐

//接收
typedef struct VisionDataRX  
{
  uint8_t header ;  //包头

  uint8_t state  ;  //状态位

	  /*
     * ## 状态位，指示自瞄运行状态 ##
     *  ______________________________________________________
     * | BIT | DESCRIBE      | SET 1      | SET 0   | DEFAULT |
     * |  0  | auto aim      | enable     | disable | disable | 自瞄是否可用 | 自瞄可用时，每帧此项都为1，方便画UI
     * |  1  | our color     | red        | blue    | blue    | 当前己方颜色，用于电控核验数据
     * |  2  | tracking      | yes        | no      | no      | 是否正在跟踪（建议在操作手按下右键但tracking为no的情况下，在UI标明）
     * | 3&4 | aim type      | 0 0 = car (default)  |         | 当前跟踪目标
     * | 3&4 | aim type      | 1 0 = outpost        |         | 非吊射模式锁定基地时，改标志位为：0 0
     * | 3&4 | aim type      | 0 1 = lob            |         | 吊射和能量机关功能需操作手启动。启动后无论是否跟踪到目标，该项均为启动
     * | 3&4 | aim type      | 1 1 = rune           |         | 的模式。
     * |  5  | reserve       |            |         |         |
     * |  6  | reserve       |            |         |         |
     * |  7  | reserve       |            |         |         |
     */

  float PitchAngleTarget ;
  float YawAngleTarget ;

  uint8_t fireControl  ; //开火位
	  /*
     * ## 开火位 ##
     * 注意：只有当开火位为 0xff 时才表示开火
     * 注意：开火位不会只发送一次，当自瞄认为当前处于适宜状态下，该开位均为 0xff
     *
     * Fire     | 0xff
     * Not Fire | Other Num
     * Default  | Not Fire
     *
     */

} VisionDatas;

union  VisionDate 
{
  VisionDatas VisionRxData ; 
  uint8_t     Rxdata[10] ; 
};


//发送

typedef struct VisionDataTX {
  uint8_t packHead;	// 包头
  uint8_t OurColor;	// 我方颜色 == 'R'(red) | 'B'(blue)
  uint8_t Command;	// 命令位，对自瞄进行控制
    /*
     * ## 命令位，对自瞄进行控制 ##
     *  __________________________________________
     * | BIT | DESCRIBE  | SET 1 | SET 0 | DEFAULT |
     * |  0  | reserve   |       |       |         |
     * |  1  | reserve   |       |       |         |
     * |  2  | reserve   |       |       |         |
     * |  3  | reserve   |       |       |         |
     * |  4  | reserve   |       |       |         |
     * |  5  | reserve   |       |       |         |
     * |  6  | reserve   |       |       |         |
     * |  7  | reserve   |       |       |         |
     */

  float PitchAngle;	// pitch轴当前值
  float  YawAngle;		// yaw轴当前值（角度）
  float  ShootSpeed;	// 弹速
} visionTX;

typedef struct VisionDataAll {
  union VisionDate RXData;
  visionTX TXData;
} VisionConnect;

#pragma pack() //恢复默认对齐

enum aim_type //接受视觉正在瞄准的类型 
{
  CAR = 0 ,   //机器人 
  OUT_POST ,  //前哨站
  RUME ,      //能量机关
};

typedef struct
{
	uint8_t aoto_aim_state;	// 自瞄状态 不能跟踪(0) | 能跟踪(1)
	uint8_t our_color;			// 我方颜色 'R'(red) | 'B'(blue)
	uint8_t tracking;				// 是否锁到敌人 0(no) | 1(yes)
	uint8_t aim_type;				// 从视觉接收的 正在自瞄的目标类型 enum类型
} VisionState;

extern VisionConnect vision1;
extern VisionState visionState;

void Vision_ConectINIT( VisionConnect *conect , uint8_t OurColor );
void VisionConnectUpdateTX(VisionConnect *connect, uint8_t OurColor, float yawAngle,
                           float pitchAngle, float shootSpeed) ;
void VisionConnectSend(VisionConnect *connnect) ;
void VisionGetState(VisionConnect* connect, VisionState* visionState); 
                         
#endif 
