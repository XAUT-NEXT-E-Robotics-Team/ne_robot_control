#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

/**
 * @brief 机器人指令应用初始化
 * 
 */
void RobotCmdInit(void);


/* ROBOT核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RoBotCmdTask(void);

#endif // !ROBOT_CMD_H
