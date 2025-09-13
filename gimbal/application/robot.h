#ifndef ROBOT_H
#define ROBOT_H 

/**
 * @brief 机器人初始化函数，在main函数中调用,是main函数中唯一调用的初始化函数
 * 
 */
void robot_init(void);

/**
* @brief ROBOT底盘，云台，发射 ，控制 任务（机器人的核心）
 * 
 */
void RobotTask(void);


#endif 
