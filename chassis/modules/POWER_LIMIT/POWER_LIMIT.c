#include "POWER_LIMIT.h"
#include "Slope_Plan.h"
Powerlimit_e Powerlimit ;
void Powerlimit_Init( Powerlimit_e * powerlimit)
{
 powerlimit->T_coff = 1.99688994e-6f ; // (20/16384)*(0.3)*(187/3591)/9.55
 powerlimit->K2 = 1.453e-07 ;  // k1
 powerlimit->a = 1.23e-07 ;    // k2
}

   float chassis_total_power = 0;
   float errorConfidence = 0.5; // 误差权重，误差越大，修正误差越快
   float initial_total_power = 0;                      //4电机的总功率  
   float spd_total_err = 0;                           //速度的差值总和
   float initial_give_power  = 0;                     // initial power from PID calculation PID计算后的功率限制
   float initial_total_power_last = 0;                 //4电机上一次的总功率
   float scaled_give_power = 0;                        //功率衰减系数
   float powerWeight_Prop = 0.0f;
   float spd_err =0.0f;


/**
 * @brief  西交利物浦大学+香港科技大学功率限制移植
 * @param  motor: 电机结构体
 * @return fp32: 功率限制后的电流值
 */
float powerlimit_LVP_HK_pro(DJIMotorInstance * motor , float power ) {
  uint16_t max_power_limit = power;                          // 裁判系统功率限制
  static float spd_total_err_last = 0;                       //速度上一次的差值总和
  float constant = 4.081f;                                   //静态损耗
//  static float chassis_total_power = 0;
//  static float errorConfidence = 0.5; // 误差权重，误差越大，修正误差越快
//  static float initial_total_power = 0;                      //4电机的总功率  
//  static  float spd_total_err = 0;                           //速度的差值总和
//  static  float initial_give_power  = 0;                     // initial power from PID calculation PID计算后的功率限制
//  static float initial_total_power_last = 0;                 //4电机上一次的总功率
//  static float scaled_give_power = 0;                        //功率衰减系数
//  static float powerWeight_Prop = 0.0f;
//  static float spd_err =0.0f;
	
  spd_err =  absf( motor->motor_controller.speed_PID.Ref/10.0f - motor->measure.speed_rpm ); // ->target - motor->realSpeedF

  float toque_coefficient = 1.99688994e-6f;  // (20/16384)*(0.3)*(187/3591)/9.55
  float a = 1.23e-07;                        // k1
  float k2 = 1.453e-07;                      // k2

  static uint8_t count = 0;  
	//计算单个电机的功率
	initial_give_power = motor->motor_controller.speed_PID.Output * toque_coefficient * motor->measure.speed_rpm
                       + k2 * motor->measure.speed_rpm * motor->measure.speed_rpm
                       + a * motor->motor_controller.speed_PID.Output * motor->motor_controller.speed_PID.Output ;
	
  if (initial_give_power >= 0) {  // negative power not included (transitory)
    initial_total_power += initial_give_power;
    spd_total_err += spd_err;
  }
  count++;

  if (count == 4) { //求出4个电机的总和
    count = 0;

    initial_total_power_last = initial_total_power;
    chassis_total_power = initial_total_power;  //
    initial_total_power = 0;

    spd_total_err_last = spd_total_err;
    spd_total_err = 0;
  }

  if (initial_total_power_last > max_power_limit)  // determine if larger than max power
  {


    if (spd_total_err_last > ERROR_POWER_DISTRIBUTION_SET)
    {
        errorConfidence = 1.0f;
    }
    else if (spd_total_err_last > PROP_POWER_DISTRIBUTION_SET)
    {
      errorConfidence = (spd_total_err_last - PROP_POWER_DISTRIBUTION_SET) / (ERROR_POWER_DISTRIBUTION_SET - PROP_POWER_DISTRIBUTION_SET);
      if (errorConfidence < 0.0f) {
        errorConfidence = 0.0f;
      }
        if (errorConfidence > 1.0f) {
        errorConfidence = 1.0f;
        }
    }
    else
    {
        errorConfidence = 0.0f;
    }

    float powerWeight_Error = spd_err / spd_total_err_last;                  // 误差权重
    powerWeight_Prop = initial_give_power / initial_total_power_last;  // 功率权重
    float powerWeight =
      errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;

    scaled_give_power = max_power_limit * powerWeight;  // get scaled power
    if (scaled_give_power < 0) {
      return motor->motor_controller.speed_PID.Output;
    }

    float b = toque_coefficient * motor->measure.speed_rpm;
    float c = k2 * motor->measure.speed_rpm * motor->measure.speed_rpm - scaled_give_power ;

    if (motor->motor_controller.speed_PID.Output > 0)  // Selection of the calculation formula according
                                // to the direction of the original moment
    {
      float temp = (-b + sqrt(b * b - 4 * a * c))
                  / (2 * a);  ////////////////////////////////////////////////////
      if (temp > 16000) {
        return 16000;
      }
      else
        return temp;
    }
    else {
      float temp = (-b - sqrt(b * b - 4 * a * c))
                  / (2 * a);  ////////////////////////////////////////////////////
      if (temp < -16000) {
        return -16000;
      }
      else
        return temp;
    }
  }
  else {
    return motor->motor_controller.speed_PID.Output;
  }
}
