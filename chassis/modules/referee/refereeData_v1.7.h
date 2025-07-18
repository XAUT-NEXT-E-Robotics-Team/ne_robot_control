// 该文件是裁判系统对具体的数据进行定义与封装
#ifndef REFEREEDATA_H
#define REFEREEDATA_H
#include "main.h"
#include "protocol.h"

/**
 * @brief  0x0001
 * 比赛状态数据@3HZ
 */
typedef __packed struct {
  uint8_t game_type : 4;       // 比赛类型
  uint8_t game_progress : 4;   // 比赛阶段
  uint16_t stage_remain_time;  // 当前阶段剩余时间(s)
  uint64_t SyncTimeStamp;  // UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} game_state_t;

// 比赛类型代号
typedef enum {
  TYPE_7V7 = 1,     // 超级对抗赛
  TYPE_SINGLE = 2,  // 单项挑战赛
  TYPE_ICRA = 3,    // 人工智能挑战赛
  TYPE_3V3 = 4,     // 联盟赛3V3
  TYPE_1V1 = 5,     // 联盟赛1V1
} game_type_t;

// 比赛阶段代号
typedef enum {
  PROGRESS_UNSTART = 0,      // 未开始
  PROGRESS_PREPARE = 1,      // 准备阶段
  PROGRESS_SELFCHECK = 2,    // 自捡阶段
  PROGRESS_5sCOUNTDOWN = 3,  // 5秒倒计时
  PROGRESS_BATTLE = 4,       // 比赛中
  PROGRESS_CALCULATING = 5,  // 比赛结算中
} game_progress_t;

/**
 * @brief  0x0002
 * 比赛结果数据
 */
typedef __packed struct  // 0002
{
  uint8_t winner;
} game_result_t;

// 比赛结果代号
typedef enum {
  WINNER_NULL = 0,  // 平局
  WINNER_RED = 1,   // 红方胜利
  WINNER_BLUE = 2,  // 蓝方胜利
} game_winner_t;

/**
 * @brief  0x0003
 * 机器人血量数据@3HZ
 */
typedef __packed struct {
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t reserved1;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t reserved2;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} game_robot_HP_t;

/**
 * @brief  0x0101
 * 场地事件数据@3HZ
 */
typedef __packed struct {
  uint8_t HP_ONLY : 1;  // 己方与兑换区不重叠的补给区占领状态，1 为已占领
  uint8_t HP_CONVERSION : 1;  // 己方与兑换区重叠的补给区占领状态，1 为已占领
  uint8_t HP : 1;  // 己方补给区的占领状态，1 为已占领（仅 RMUL 适用）

  uint8_t ENERGY_SMALL_EN : 1;  // 己方小能量机关的激活状态，1 为已激活
  uint8_t ENERGY_LARGE_EN : 1;  // 己方大能量机关的激活状态，1 为已激活

  uint8_t HILL_CENTER_N2 : 2;  // 己方中央高地的占领状态，1为被己方占领，2为被对方占领
  uint8_t HILL_TRAP : 2;  // 己方梯形高地的占领状态，1为被己方占领，2为被对方占领

  uint16_t The_last_time_hit : 9;  // 飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
  uint8_t The_outpost_base : 3;  // 飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为0，1为击中前哨站，2为击中基地固定目标，3为击中基地随机固定目标，4 为击中基地随机移动目标
  uint8_t center_gain : 2;  // 中心增益点的占领情况，0 为未被占领，1 为被己方占领，2 为被对方占领，3为被双方占领。
  uint16_t UNKNOWN : 9;      // 预留
} event_data_t;

// 补弹机器人代号
typedef enum {
  NO_SUPPLY_ROBOT = 0,     // 当前无机器人补弹
  RED_HERO_SUPPLY = 1,     // 红方英雄机器人补弹
  RED_N3_SUPPLY = 3,       // 红方步兵机器人补弹
  RED_N4_SUPPLY = 4,       // 红方步兵机器人补弹
  RED_N5_SUPPLY = 5,       // 红方步兵机器人补弹
  BLUE_HERO_SUPPLY = 101,  // 蓝方英雄机器人补弹
  BLUE_N3_SUPPLY = 103,    // 蓝方步兵机器人补弹
  BLUE_N4_SUPPLY = 104,    // 蓝方步兵机器人补弹
  BLUE_N5_SUPPLY = 105,    // 蓝方步兵机器人补弹
} supply_robot_id_t;

// 补弹口状态代号
typedef enum {
  SUPPLY_OFF = 0,        // 关闭
  SUPPLY_PREPARING = 1,  // 弹丸准备中
  SUPPLY_READY = 2,      // 弹丸释放
} supply_state_t;

typedef enum {
  Replenishment_quantity_A = 50,   // 补弹数量50颗弹丸
  Replenishment_quantity_B = 100,  // 补弹数量100颗弹丸
  Replenishment_quantity_C = 150,  // 补弹数量150颗弹丸
  Replenishment_quantity_D = 200,  // 补弹数量200颗弹丸
} supply_projectile_num;

/**
 * @brief  0x0104
 * 裁判警告数据，己方判罚/判负时触发发送
 */
typedef __packed struct {
  uint8_t level;               // 判罚等级
  uint8_t offending_robot_id;  // 违规机器人 ID
  uint8_t count;  // 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。
} referee_warning_t;

// 判罚等级代号
typedef enum {
  TWO_YELLOW_WARINING = 1,  // 双方黄牌
  YELLOW_WARINING = 2,      // 黄牌
  RED_WARINING = 3,         // 红牌
  FAIL_WARINING = 4,        // 判负
} warning_level_t;

// 违规机器人代号
typedef enum {
  NO_WARNED_ROBOT = 0,     // 当前无机器人判罚
  RED_HERO_WARNED = 1,     // 红方英雄机器人判罚
  RED_N3_WARNED = 3,       // 红方步兵机器人判罚
  RED_N4_WARNED = 4,       // 红方步兵机器人判罚
  RED_N5_WARNED = 5,       // 红方步兵机器人判罚
  BLUE_HERO_WARNED = 101,  // 蓝方英雄机器人判罚
  BLUE_N3_WARNED = 103,    // 蓝方步兵机器人判罚
  BLUE_N4_WARNED = 104,    // 蓝方步兵机器人判罚
  BLUE_N5_WARNED = 105,    // 蓝方步兵机器人判罚
} robot_warning_id_t;

/**
 * @brief  0x0105
 * 飞镖发射时间数据，@3Hz
 */
typedef __packed struct {
  uint8_t dart_remaining_time;  // 己方飞镖发射剩余时间，单位：秒
  uint8_t
    The_own_dart : 3;  // 最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机固定目标，4 为击中基地随机移动目标
  uint8_t Total_numbe : 3;  // 对方最近被击中的目标累计被击中计数，开局默认为0，至多为4
  uint8_t
    The_hit_the_target : 2;  // 飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为0，选中基地固定目标为1，选中基地随机目标为2，选中基地随机移动目标为 3
  uint16_t rest : 8;         // 保留
} dart_remaining_time_t;

/**
 * @brief  0x0201
 * 机器人性能体系数据，@10HZ
 */
typedef __packed struct {
  uint8_t robot_id;                            // 本机器人ID
  uint8_t robot_level;                         // 机器人等级
  uint16_t current_HP;                         // 机器人当前血量
  uint16_t maximum_HP;                         // 机器人血量上限
  uint16_t shooter_barrel_cooling_value;       // 机器人枪口热量每秒冷却值
  uint16_t shooter_barrel_heat_limit;          // 机器人枪口热量上限
  uint16_t chassis_power_limit;                // 机器人底盘功率上限
  uint8_t power_management_gimbal_output : 1;  // gimbal口输出：0为无输出，1为24V输出
  uint8_t power_management_chassis_output : 1;  // ：chassis口输出：0为无输出，1为24V输出
  uint8_t power_management_shooter_output : 1;  // shooter口输出：0为无输出，1为24V输出
} robot_state_t;

/**
 * @brief  0x0202
 * 实时功率热量数据，@50Hz
 */
typedef __packed struct {
  uint16_t reserved1;  // 电源管理模块 chassis 口输出电压（单位：mV）
  uint16_t reserved2;  // 电源管理模块 chassis 口输出电流（单位：mA）
  float reserved3;       // 底盘功率（单位：W）
  uint16_t buffer_energy;    // 缓冲能量（单位：J）
  uint16_t shooter_17mm_1_barrel_heat;  // 第 1 个 17mm 发射机构的枪口热量
  uint16_t shooter_17mm_2_barrel_heat;  // 第 2 个 17mm 发射机构的枪口热量
  uint16_t shooter_42mm_barrel_heat;    // 42mm 发射机构的枪口热量
} power_heat_data_t;

/**
 * @brief  0x0203
 * 机器人位置数据，@10Hz
 */
typedef __packed struct {
  float x;      // 本机器人位置x坐标，单位：m
  float y;      // 机器人位置y坐标，单位：m
  float angle;  // 本机器人测速模块的朝向，单位：度。正北为0度
} game_robot_pos_t;

/**
 * @brief  0x0204
 * 机器人增益数据，@3Hz
 */
typedef __packed struct {
  uint8_t recovery_buff;  // 机器人回血增益（百分比，值为 10 意为每秒回复
  // 10%最大血量）
  uint8_t cooling_buff;  // 机器人枪口冷却倍率（直接值，值为 5 意味着 5 倍冷却）
  uint8_t defence_buff;  // 机器人防御增益（百分比，值为 50 意为 50%防御增益）
  uint8_t vulnerability_buff;  // 机器人负防御增益（百分比，值为30表示-30%防御增益）
  uint16_t attack_buff;  // 机器人攻击增益（百分比，值为 50 意为 50%攻击增益）
  uint8_t remaining_energy_50 : 1;// 在剩余能量≥50%时为 1，其余情况为 0
  uint8_t remaining_energy_30 : 1;// 在剩余能量≥30%时为 1，其余情况为 0
  uint8_t remaining_energy_15 : 1;// 在剩余能量≥15%时为 1，其余情况为 0
  uint8_t remaining_energy_5  : 1;// 在剩余能量≥5%时为 1，其余情况为 0
  uint8_t remaining_energy_1  : 1;// 在剩余能量≥1%时为 1，其余情况为 0
} robot_buff_t;


/**
 * @brief  0x0206
 * 伤害状态数据，伤害发生后发送
 */
typedef __packed struct {
  uint8_t armor_id : 4;             // 当扣血原因为装甲模块或测速模块时，该 4bit
                                    // 组成的数值为装甲模块或测速模块的 ID
                                    // 编号；其他原因扣血时，该数值为 0
  uint8_t HP_deduction_reason : 4;  // 血量变化类型
} hurt_data_t;

// 扣血原因代号
typedef enum {
  AMMO_HIT = 0,            // 装甲被弹丸攻击扣血
  SYSTEM_OFFLINE = 1,      // 裁判系统重要模块离线扣血
  SHOOT_OVER_SPEED = 2,    // 射击初速度超限扣血
  SHOOT_OVER_HEAT = 3,     // 枪口热量超限扣血
  CHASSIS_OVER_POWER = 4,  // 底盘功率超限扣血
  CRASH_HIT = 5,           // 装甲模块受到撞击扣血
} hurt_reason_t;

/**
 * @brief  0x0207
 * 实时射击数据，弹丸发射后发送
 */
typedef __packed struct {
  uint8_t bullet_type;     // 弹丸类型：1:17mm 2:42mm
  uint8_t shooter_number;  // 发射机构
  // ID,1:第一个17mm发射机构，2:第二个17mm发射机构，3:42mm发射机构
  uint8_t launching_frequency;  // 弹丸射频（单位：Hz）
  float initial_speed;          // 弹丸初速度（单位：m/s）
} shoot_data_t;

/**
 * @brief  0x0208
 * 允许发弹量，@10Hz
 */
typedef __packed struct {
  uint16_t projectile_allowance_17mm;  // 17mm 弹丸允许发弹量
  uint16_t projectile_allowance_42mm;  // 42mm 弹丸允许发弹量
  uint16_t remaining_gold_coin;        // 剩余金币数量
} projectile_allowance_t;

/**
 * @brief  0x0209
 * 机器人 RFID 状态，@3Hz
 */
typedef __packed struct {
  uint8_t self_base_buff_area : 1;          // 己方基地增益点
  uint8_t self_circle_hill_buff_area : 1;   // 己方环形高地增益点
  uint8_t enemy_circle_hill_buff_area : 1;  // 对方环形高地增益点
  uint8_t self_R3_B3_buff_area : 1;         // 己方 R3/B3 梯形高地增益点
  uint8_t enemy_R3_B3_buff_area : 1;        // 对方 R3/B3 梯形高地增益点
  uint8_t self_R4_B4_buff_area : 1;         // 己方 R4/B4 梯形高地增益点
  uint8_t enemy_R4_B4_buff_area : 1;        // 对方 R4/B4 梯形高地增益点
  uint8_t self_energy_en_area : 1;          // 己方能量机关激活点
  uint8_t self_fly_buff_before_area : 1;   // 己方飞坡增益点（靠近己方一侧飞坡前）
  uint8_t self_fly_buff_after_area : 1;    // 己方飞坡增益点（靠近己方一侧飞坡后）
  uint8_t enemy_fly_buff_before_area : 1;  // 对方飞坡增益点（靠近己方一侧飞坡前）
  uint8_t enemy_fly_buff_after_area : 1;   // 对方飞坡增益点（靠近己方一侧飞坡后）
  uint8_t self_outpost_buff_area : 1;      // 己方前哨站增益点
  uint8_t self_medical_buff_area : 1;      // 己方补血点（检测到任一均视为激活）
  uint8_t self_robot_N7_buff_area : 1;     // 己方哨兵巡逻区
  uint8_t enemy_robot_N7_buff_area : 1;    // 对方哨兵巡逻区
  uint8_t self_resources_buff_area : 1;    // 己方大资源岛增益点
  uint8_t enemy_resources_buff_area : 1;   // 对方大资源岛增益点

  uint8_t self_conversion_area : 1;  // 己方兑换区
  uint8_t control_area : 1;          // 中心增益点（仅RMUL适用）
  uint16_t unknown : 12;             // 保留
} rfid_status_t;

/**
 * @brief  0x020A
 * 飞镖选手端指令数据，飞镖闸门上线后@10Hz
 */
typedef __packed struct {
  uint8_t dart_launch_opening_status;  // 当前飞镖发射站的状态
                                       // 1:关闭，2：正在开启或关闭中，3：开启
  uint8_t reserved;                    // 保留位
  uint16_t target_change_time;  // 切换打击目标时的比赛剩余时间，单位：s，无未切换动作默认为
  // 0。
  uint16_t latest_launch_cmd_time;  // 最后一次操作手确定发射指令时的比赛剩余时间，单位：s，初始值为
                                    // 0。
} dart_client_cmd_t;

/**
 * @brief  0x020B
 * 地面机器人位置数据，@1Hz
 */
typedef __packed struct {
  float hero_x;        // 己方英雄机器人位置 x 轴坐标，单位：m
  float hero_y;        // 己方英雄机器人位置 y 轴坐标，单位：m
  float engineer_x;    // 己方工程机器人位置 x 轴坐标，单位：m
  float engineer_y;    // 己方工程机器人位置 y 轴坐标，单位：m
  float standard_3_x;  // 己方 3 号步兵机器人位置 x 轴坐标，单位：m
  float standard_3_y;  // 己方 3 号步兵机器人位置 y 轴坐标，单位：m
  float standard_4_x;  // 己方 4 号步兵机器人位置 x 轴坐标，单位：m
  float standard_4_y;  // 己方 4 号步兵机器人位置 y 轴坐标，单位：m
  float reserved1;  // 己方 5 号步兵机器人位置 x 轴坐标，单位：m
  float reserved2;  // 己方 5 号步兵机器人位置 y 轴坐标，单位：m
} ground_robot_position_t;

/**
 * @brief  0x020C
 * 雷达标记进度数据，@1Hz
 */
typedef __packed struct {
  uint8_t mark_hero_progress : 1;        //在对应机器人被标记进度≥100 时发送 1，被标记进度<100 时发送 0。
  uint8_t mark_engineer_progress : 1;     
  uint8_t mark_standard_3_progress : 1;   
  uint8_t mark_standard_4_progress : 1;   
  uint8_t mark_standard_5_progress : 1;   
  uint8_t mark_sentry_progress;       
} radar_mark_data_t;

/**
 * @brief  0x020D
 *
 */
typedef __packed struct {
  uint16_t
    Sentinel_orders_quantity : 11;  // ：除远程兑换外，哨兵成功兑换的发弹量，开局为0，在哨兵成功兑换一定发弹量后，该值将变为哨兵成功兑换的发弹量值。
  uint8_t
    The_number_redeems_ammunition : 4;  // 哨兵成功远程兑换发弹量的次数，开局为0，在哨兵成功远程兑换发弹量后，该值将变为哨兵成功远程兑换发弹量的次数。
  uint8_t
    The_number_redeem_health_remotely : 4;  // ：哨兵成功远程兑换血量的次数，开局为0，在哨兵成功远程兑换血量后，该值将变为哨兵成功远程兑换血量的次数。
  uint16_t esrt : 13;                       // ：保留
} sentry_info_t;

/**
 * @brief  0x020E
 *
 */
typedef __packed struct {
  uint8_t
    Radar_doubling : 2;  // ：雷达是否拥有触发双倍易伤的机会，开局为0，数值为雷达拥有触发双倍易伤的机会，至多为2
  uint8_t error_doubling : 1;  // 对方是否正在被触发双倍易伤
  // 0：对方未被触发双倍易伤
  // 1：对方正在被触发双倍易伤
  uint8_t rset : 5;  // 保留
} radar_info_t;

/**
 * @brief  0x0301
 * 机器人交互数据，发送方触发发送，频率上限为 10Hz
 */
typedef __packed struct {
  uint16_t data_cmd_id;  // 子内容 ID ,需为开放的子内容 ID
  uint16_t sender_id;    // 发送者 ID ,需与自身 ID 匹配，ID 编号详见附录
  uint16_t receiver_id;  // 接收者
  // ID,仅限己方通信,需为规则允许的车间通信接收者,若接收者为选手端，则仅可发送至发送者对应的选手端,ID
  // 编号详见附录
  uint8_t user_data[20];  // 内容数据段,数组长度最大为 113
} robot_interaction_data_t;

/**
 * @brief  0x0100
 *
 */
typedef __packed struct {
  uint8_t delete_type;  // 删除操作
  uint8_t layer;        // 图层数
} interaction_layer_delete_t;

/**
 * @brief  0x0101
 *
 */
typedef __packed struct {
  uint8_t figure_name[3];     // 图形名 在图形删除、修改等操作中，作为索引
  uint32_t operate_tpye : 3;  // bit 0-2：图形操作
  // 0：空操作
  // 1：增加
  // 2：修改
  // 3：删除
  uint32_t figure_tpye : 3;  // bit 3-5：图形类型
  //  0：直线
  //  1：矩形
  //  2：正圆
  //  3：椭圆
  //  4：圆弧
  //  5：浮点数
  //  6：整型数
  //  7：字符
  uint32_t layer : 4;  // ：图层数（0~9）
  uint32_t color : 4;  // bit 10-13：颜色
  //  0：红/蓝（己方颜色）
  //  1：黄色
  //  2：绿色
  //  3：橙色
  //  4：紫红色
  //  5：粉色
  //  6：青色
  //  7：黑色
  //  8：白色
  uint32_t details_a : 9;  // 根据绘制的图形不同，含义不同，详见“表 2-26 图形细节参数说明”
  uint32_t details_b : 9;
  uint32_t width : 10;    // 线宽，建议字体大小与线宽比例为10：1
  uint32_t start_x : 11;  // 起点/圆心x坐标
  uint32_t start_y : 11;  // ：起点/圆心y坐标
  uint32_t details_c : 10;  // 根据绘制的图形不同，含义不同，详见“表 2-26 图形细节参数说明”
  uint32_t details_d : 11;
  uint32_t details_e : 11;
} interaction_figure_t;

/**
 * @brief  0x0102
 *
 */
typedef __packed struct {
  interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

/**
 * @brief  0x0103
 *
 */
typedef __packed struct {
  interaction_figure_t interaction_figure[5];  // 图形1 与0x0101的数据段相同
                                               // 图形2 与0x0101的数据段相同
                                               // 图形3 与0x0101的数据段相同
                                               // 图形4 与0x0101的数据段相同
                                               // 图形5 与0x0101的数据段相同
} interaction_figure_3_t;

/**
 * @brief  0x0104
 *
 */
typedef __packed struct {
  interaction_figure_t interaction_figure[7];  // 图形1 2 3 4 5 6 7 与0x0101的数据段相同
} interaction_figure_4_t;

/**
 * @brief  0x0110
 *
 */
// typedef __packed struct {
//   graphic_data_struct_t grapic_data_struct;  // 数据的内容ID
//   uint8_t data[30];                          // 发送者的ID 需要校验发送者的ID正确性
//	接收者的ID
//  需要校验接收者的ID正确性，仅支持发送机器人对应的选手端
//  字符配置 详见图形数据介绍
// } ext_client_custom_character_t;

/**
 * @brief  0x0120
 *
 */
typedef __packed struct {
  uint8_t Sentinel_resurrection : 1;  // S：哨兵机器人是否确认复活
  //  0表示哨兵机器人确认不复活，即使此时哨兵的复活读条
  // 已经完成
  //  1表示哨兵机器人确认复活，若复活读条完成将立即复活
  uint8_t Sentinel_redemption_resurrection : 1;  // 哨兵机器人是否确认兑换立即复活
  uint16_t
    Sentinel_redemption_invoice_volume : 11;  // ：哨兵将要兑换的发弹量值，开局为0，修改此值后，哨兵在补血点即可兑换允许发弹量。
  uint8_t Sentinel_times : 4;                 // 哨兵远程兑换发弹量的请求次数
  uint8_t Sentinel_hp_times : 4;              // ：哨兵远程兑换血量的请求次数
  uint16_t Unknown : 11;               // 保留
} sentry_cmd_t;

/**
 * @brief  0x0121
 *
 */
typedef __packed struct {
  uint8_t radar_cmd;  // 雷达是否确认触发双倍易伤
                      // 开局为0，修改此值即可请求触发双倍易伤，若此时雷达拥有
                      // 触发双倍易伤的机会，则可触发。
                      // 此值的变化需要单调递增且每次仅能增加1，否则视为不合
                      // 法.
                      // 示例：此值开局仅能为0，此后雷达可将其从0修改至1，若雷
                      // 达拥有触发双倍易伤的机会，则触发双倍易伤。此后雷达可将
                      // 其从1修改至2，以此类推。
                      // 若雷达请求双倍易伤时，双倍易伤正在生效，则第二次双倍易
                      // 伤将在第一次双倍易伤结束后生效。
} radar_cmd_t;

/**
 * @brief  0x0303
 *
 */
typedef __packed struct {
  float target_position_x;  // 目标位置x轴坐标，单位m 当发送目标机器人ID时，该值为0
  float target_position_y;  // 目标位置y轴坐标，单位m 当发送目标机器人ID时，该值为0
  uint8_t cmd_keyboard;     // 云台手按下的键盘按键通用键值 无按键按下，则为0
  uint8_t target_robot_id;  // 对方机器人ID 当发送坐标数据时，该值为0
  uint8_t cmd_source;       // 信息来源ID 信息来源的ID，ID对应关系详见附录
} map_command_t;

/**
 * @brief  0x0305
 *
 */
typedef __packed struct {
  uint16_t target_robot_id;  // 目标机器人ID
  float target_position_x;   // 目标x位置坐标，单位：m
  float target_position_y;   // 目标y位置坐标，单位：m
} map_robot_data_t;

/**
 * @brief  0x0307
 * 键鼠遥控数据，@30Hz
 */
typedef __packed struct {
  uint8_t intention;  // 1：到目标点攻击
  // 2：到目标点防守 -
  // 3：移动到目标点
  uint16_t start_position_x;  // 路径起点x轴坐标，单位：dm
  uint16_t start_position_y;  // 路径起点y轴坐标，单位：dm
  int8_t delta_x[49];         // 路径点x轴增量数组，单位：dm
  int8_t delta_y[49];         // 路径点y轴增量数组，单位：dm
  uint16_t sender_id;         // 送者ID
} map_data_t;

/**
 * @brief  0x0308
 *
 */
typedef __packed struct {
  uint16_t sender_id;     // 发送者的ID
  uint16_t receiver_id;   // 接收者的ID
  uint8_t user_data[30];  // 字符
} custom_info_t;

/**
 * @brief  0x0302
 *操作手可使用自定义控制器通过图传链路向对应的机器人发送数据。
 */
typedef __packed struct {
  uint8_t data[30];
} custom_robot_data_t;

/**
 * @brief  0x0306
 *
 */
typedef __packed struct {
  uint16_t key_value;
  uint16_t x_position : 12;
  uint16_t mouse_left : 4;
  uint16_t y_position : 12;
  uint16_t mouse_right : 4;
  uint16_t reserved;
} custom_client_data_t;

/**
 * @brief  0x0309
 *机器人可通过图传链路向对应的操作手选手端连接的自定义控制器发送数据（RMUL 暂不适用）。
 */
typedef __packed struct
{
uint8_t data[1];
}robot_custom_data_t;
/**
 * @brief  0x0304
 * 键鼠遥控数据，@30Hz
 */
typedef __packed struct {
  int16_t mouse_x;           // 鼠标 x 轴移动速度，负值标识向左移动
  int16_t mouse_y;           // 鼠标 y 轴移动速度，负值标识向下移动
  int16_t mouse_z;           // 鼠标滚轮移动速度，负值标识向后滚动
  int8_t left_button_down;   // 鼠标左键是否按下：0 为没按下；1 为按下
  int8_t right_button_down;  // 鼠标右键是否按下：0 为没按下，1 为按下
  uint8_t KEY_W : 1;         // W 键,0 为没按下，1 为按下
  uint8_t KEY_S : 1;         // s 键,0 为没按下，1 为按下
  uint8_t KEY_A : 1;         // A 键,0 为没按下，1 为按下
  uint8_t KEY_D : 1;         // D 键,0 为没按下，1 为按下
  uint8_t KEY_Shift : 1;     // Shift 键,0 为没按下，1 为按下
  uint8_t KEY_Ctrl : 1;      // Ctrl 键,0 为没按下，1 为按下
  uint8_t KEY_Q : 1;         // Q 键,0 为没按下，1 为按下
  uint8_t KEY_E : 1;         // E 键,0 为没按下，1 为按下
  uint8_t KEY_R : 1;         // R 键,0 为没按下，1 为按下
  uint8_t KEY_F : 1;         // F 键,0 为没按下，1 为按下
  uint8_t KEY_G : 1;         // G 键,0 为没按下，1 为按下
  uint8_t KEY_Z : 1;         // Z 键,0 为没按下，1 为按下
  uint8_t KEY_X : 1;         // X 键,0 为没按下，1 为按下
  uint8_t KEY_C : 1;         // C 键,0 为没按下，1 为按下
  uint8_t KEY_V : 1;         // V 键,0 为没按下，1 为按下
  uint8_t KEY_B : 1;         // B 键,0 为没按下，1 为按下
  uint16_t reserved;
} referee_remote_control_t;

extern game_state_t game_state;                              // 0x0001
extern game_result_t game_result;                            // 0x0002
extern game_robot_HP_t game_robot_HP;                        // 0x0003
extern event_data_t event_data;                              // 0x0101
extern referee_warning_t referee_warning;                    // 0x0104
extern dart_remaining_time_t dart_remaining_time;            // 0x0105
extern robot_state_t robot_state;                            // 0x0201
extern power_heat_data_t power_heat_data;                    // 0x0202
extern game_robot_pos_t game_robot_pos;                      // 0x0203
extern robot_buff_t robot_buff;                              // x0204
extern hurt_data_t hurt_data;                                // 0x0206
extern shoot_data_t shoot_data;                              // 0x0207
extern projectile_allowance_t projectile_allowance;          // 0x0208
extern rfid_status_t rfid_status;                            // 0x0209
extern dart_client_cmd_t dart_client_cmd;                    // 0x020A
extern ground_robot_position_t ground_robot_position;        // 0x020B
extern radar_mark_data_t radar_mark_data;                    // 0x020C
extern robot_interaction_data_t robot_interaction_data;      // 0x0301
extern referee_remote_control_t referee_remote_control;      // 0x0304
extern robot_custom_data_t robot_custom_data;			     // 0x0309

#endif
