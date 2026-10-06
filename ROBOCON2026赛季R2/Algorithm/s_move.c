/**
 ******************************************************************************
 * @file     s_move.c
 * @brief   速度规划 — Sigmoid S 型速度曲线与时间模型
 *
 *
 *      速度与时间模型（S_move_Init / S_move_calc）
 *      用 logistic 函数 y = 1/(1+e^(-x)) 在时间轴上生成 S 型速度过渡。
 *      给定初始速度 → 目标速度，在 maxTimes 个时间片内平滑逼近，无阶跃。
 *      flexible 控制曲线陡缓，domain 控制时间窗宽度，stepSpeed 控制粒度。
 *
 *      适用场景：电机启停、固定目标变速——目标已知且不变。
 *      局限：视觉目标跳变时，时间基准对不上新距离，需重算。
 *      这个局限正是 navigation.c 选择距离基速度调制的原因。
 ******************************************************************************
 */
#include "s_move.h"

/*****************************速度与时间模型*************************************/

#define SPEED_START_MIN_3508 20  // 3508电机
#define SPEED_START_MIN_3519 200 // 3508电机
#define T 2                      // 4ms计算一次

/*S型曲线速度拟合函数y=1/(1+e^(-x))*/
static void S_move_calc(CurveObjectType *motor_S, float speed_target)
{
  float power = 0.0;
  float speed = 0.0;
  if (motor_S->aTimes < motor_S->maxTimes)
  {
    motor_S->aTimes++;
  }
  power = (2 * ((float)motor_S->aTimes) - ((float)motor_S->maxTimes)) / ((float)motor_S->maxTimes / motor_S->domain);
  power = (0.0 - motor_S->flexible) * power;
  speed = 1 + expf(power);
  speed = (motor_S->targetSpeed - motor_S->startSpeed) / speed;
  /*  BUG  */
  //	if(speed_target<10.0f&&speed_target>-10.0f){
  //	   motor_S->currentSpeed=0.0;
  //	}
  motor_S->currentSpeed = speed + motor_S->startSpeed;
  if (motor_S->currentSpeed > motor_S->targetSpeed && motor_S->targetSpeed >= motor_S->startSpeed)
  {
    motor_S->currentSpeed = motor_S->targetSpeed;
  }
  if (motor_S->currentSpeed < motor_S->targetSpeed && motor_S->targetSpeed <= motor_S->startSpeed)
  {
    motor_S->currentSpeed = motor_S->targetSpeed;
  }
  /* ‘修复’BUG */
  if (motor_S->aTimes >= motor_S->maxTimes)
  {
    motor_S->maxTimes = 0;
    motor_S->aTimes = 0;
  }
  // 简易滤波
  if (motor_S->maxTimes <= 5)
  {
    motor_S->currentSpeed = motor_S->targetSpeed;
  }
}

// 时间片计算初始化
static void tim_calc(CurveObjectType *motor_S, float speed_target)
{
  uint32_t temp = 0;
  if ((fabs(motor_S->currentSpeed - motor_S->startSpeed) <= motor_S->stepSpeed) && (motor_S->maxTimes == 0))
  {
    temp = fabs(motor_S->targetSpeed - motor_S->startSpeed);
    temp = temp / motor_S->stepSpeed;

    temp = temp / T; // 确保时间片宽度

    motor_S->maxTimes = (uint32_t)(temp) + 1;
    motor_S->aTimes = 0;
  }
}

// S型曲线拟合
void S_move_Init(CurveObjectType *motor_S, float speed_target, float speed_actual, float flexible, float stepSpeed, float domain)
{

  if (motor_S->aTimes == 0)
  {
    // 初始化赋值
    motor_S->flexible = flexible; // 曲线拉伸度
    motor_S->startSpeed = speed_actual;
    motor_S->targetSpeed = speed_target;
    motor_S->stepSpeed = stepSpeed; // 时间片步长
    motor_S->currentSpeed = speed_actual;
    motor_S->domain = domain; // 曲线域

    tim_calc(motor_S, speed_target);
  }
  motor_S->currentSpeed = speed_actual;
  // 进行S速度解算
  S_move_calc(motor_S, speed_target);
  /*  ‘修复’BUG  */
  /*确保函数计算在电机运动之上，speed=20为3508电机本次PID所需最低速度*/
  if (fabs(motor_S->currentSpeed) >= SPEED_START_MIN_3519 && fabs(speed_actual) <= 9)
  {
    motor_S->maxTimes = 0;
    motor_S->aTimes = 0;
  }
}

/*****************************速度与距离模型*************************************/

// SigmoidDist_ParamPkg S_Dist_Pkg;
// SigmoidDist  S_Dist;

// void S_Dist_Pkg_Init(void){
//   S_Dist_Pkg.C=0.5;
//	S_Dist_Pkg.coeff_High=0.75;
//   S_Dist_Pkg.coeff_Low=1;
//	S_Dist_Pkg.coeff_Vo=0.1;
//   S_Dist_Pkg.k=6;
// }
//
// int16_t SigmoidDist_Vmax_calc(int16_t dist)
//{
//	int16_t Vmax;
//   int x=dist*0.5;
//
//	if(abs(x)<=100)
//	{Vmax=(0.2*log10(x)+0.2*log10(x)*log10(x))*S_Dist_Pkg.coeff_Low;}
//
//   if(abs(x)>100&&abs(x)<=2000)
//	{Vmax=(0.01756*x+2.445)*S_Dist_Pkg.coeff_High+S_Dist_Pkg.C;}
//
//   if(abs(x)>2000)
//	{Vmax=(0.01756*2000+2.445)*S_Dist_Pkg.coeff_High+S_Dist_Pkg.C;}
//
//	S_Dist.Vmax=Vmax;
// }

// void 	SigmoidDist_Task(void){
//
//	S_Dist.Vo=S_Dist_Pkg.coeff_Vo*S_Dist.Vmax;
//
//	float power=0.0;
//   float speed=0.0;
//
//	if(S_Dist.Dist_error<S_Dist.Dist*0.5){
//   power=(2.0f*S_Dist.Dist_error-S_Dist.Dist*0.5)/((float)S_Dist.Dist*0.5/S_Dist.domain);
//   power=(0.0-S_Dist_Pkg.k)*power;
//	speed=1+expf(power);
//   speed=(S_Dist.Vmax-S_Dist.Vo)/speed;
//	}
//	if(S_Dist.Dist_error>=S_Dist.Dist*0.5){
//	power=(2.0f*(S_Dist.Dist_error-S_Dist.Dist*0.5f)-S_Dist.Dist*0.5)/((float)S_Dist.Dist*0.5/S_Dist.domain);
//   power=(0.0-S_Dist_Pkg.k)*power;
//	speed=1+expf(power);
//   speed=(0-S_Dist.Vmax)/speed;
//	}
//   S_Dist.speed=speed;
//
// }
