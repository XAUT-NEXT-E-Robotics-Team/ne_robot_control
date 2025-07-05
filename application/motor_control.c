#include "motor_control.h"

#include "dji_motor.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "general_def.h"

DJIMotorInstance *motor_test = NULL; 
float speed = 0.0f;

void MotorTestInit(){
    Motor_Init_Config_s config ={
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    config.can_init_config.tx_id = 1;
    motor_test = DJIMotorInit(&config);
}

void MotorTestControl(void)
{
    DJIMotorSetRef(motor_test, speed);
}
