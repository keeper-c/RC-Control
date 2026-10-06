#ifndef S_MOVE_H
#define S_MOVE_H

#include "main.h"
#include "pid.h"

/* 定义电机速度曲线对象 速度与时间模型 */
typedef struct CurveObject
{
  float startSpeed;   // 开始调速时的初始速度
  float currentSpeed; // 当前速度
  float targetSpeed;  // 目标速度
  float stepSpeed;    // 加速度 速度变化步长
  int32_t aTimes;     // 调速时间
  int32_t maxTimes;   // 调速跨度
  float flexible;     // S曲线拉伸度
  int domain;         // 曲线域
} CurveObjectType;
extern CurveObjectType motor_S[8];

/* 定义电机速度曲线对象 速度与距离模型 */
// typedef struct {
//   float coeff_Low;
//	float coeff_High;
//	float coeff_Vo;
//	float C;
//   float k;
//	float shift;
// }SigmoidDist_ParamPkg;
//
// typedef struct {
//   float Vmax;
//	float Vo;
//	float Dist_error;
//	float Dist;
//	float speed;
//	int domain;           //曲线域
// }SigmoidDist;
// extern SigmoidDist  S_Dist;

void S_move_calc(CurveObjectType *motor_S, float speed_target);
void tim_calc(CurveObjectType *motor_S, float speed_target);
void S_move_Init(CurveObjectType *motor_S, float speed_target, float speed_actual, float flexible, float stepSpeed, float domain);

void S_Dist_Pkg_Init(void);
int16_t SigmoidDist_Vmax_calc(int16_t dist);
void SigmoidDist_Task(void);

#endif
