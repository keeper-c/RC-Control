/**
 ******************************************************************************
 * @file    navigation.c
 * @brief   全向轮/麦轮/舵轮通用底盘速度解算与跟随算法
 *
 *  视觉/雷达世界坐标系 → 旋转变换到机器人系 → 轮系分解 → PID 跟踪 → 变阻尼削出 →
 *  信号自适应限幅 → 四轮速度输出。
 *
 * ── 核心设计：指数衰减速度调制器 ──
 *
 *  按当前误差实时计算速度系数，远快近慢、自然刹停，全程自动。
 *  效果等同梯形速度规划，无需加减速状态机，靠三个独立环节协作：
 *  
 *  [1] 指数距离衰减 — 速度轮廓
 *      coeff = kCoeffFollow × expf(kCoeffFollowExp × |误差|)
 *      远距离系数趋近 1.0（全速），靠近时指数下降，趋近零自动刹停。
 *  
 *  [2] 阻尼 + 增速 — 输出修形
 *      跟随方向：kDampingFollow 向零拉 → 刹，抑制超调。
 *      旋转方向：add_extra = |θ|×0.08 + θ²×0.00015 向外推 → 推，克服静摩擦
 *               角度大时快转到位，角度→0 增量为零自然交还给 PID。
 *
 *  [3] 信号自适应限幅 — 场景分档
 *      不同视觉阶段硬上限，防止参数在另一场景失控：
 *      - 台阶/平地夹取：跟随±6  旋转±6
 *      - KFS 接近 (Sig=10/7)：旋转±16（允许快转对准）
 *      - KFS 精调 (Sig=1)：旋转±4
 *      - 视觉对准：跟随×0.7（减速）
 *
 *  执行流水线：z_calc → 坐标系旋转 → 麦轮分解 → PID → 阻尼 → 限幅 → vPID_Input_calc 合成
 *
 ******************************************************************************
 */
#include "navigation.h"
#include "rob_core.h"
#include "rob_act.h"

#define kCoeffFollow 1.4f		  // 跟随PID基础速度系数
#define kCoeffFollowExp -0.00023f // 距离-速度指数衰减系数 25

#define kCoeffTurn 0.010f // 旋转PID速度系数 0.0141对应系数1.4， 0.0160对应系数1.2

#define kDampingTurn 0        // 旋转抑制量：将 PID 输出向零拉，等效增加阻尼防超调
#define kDampingFollow 0.3f   // 跟随抑制量：同上，麦轮跟随方向的人造阻尼

#define kCoeffOffect 2.6f

#define kPi 3.1415926535f

// 导航计算缓冲区（模块级静态，避免栈溢出）
typedef struct
{
	float follow_out[4]; // 跟随PID输出
	float turn_out[4];	 // 旋转PID输出
	float auto_out[4];	 // 自动对准PID输出
	float offset_out[4]; // 抵偿坐标PID输出
	float zero_out[4];

	float follow_raw[4]; // 跟随原始分解值
	float turn_raw[4];	 // 旋转原始分解值
	float offset_raw[4]; // 抵偿坐标原始分解值

	float combined_out[4]; // 综合输出
} NavBuf_t;

static NavBuf_t s_nav;

/*三层串联PID*/

// 抵偿坐标计算：根据机器人当前坐标和轨迹函数，计算出一个抵偿坐标，使得机器人能够沿着正确的轨迹前进
static void offset(Coordinate inst, Coordinate self, Coordinate target, Coordinate *offset);
// 机器人基准坐标系-Z轴计算
static void z_calc(Coordinate robot_current, Coordinate robot_target, Coordinate *robot_error);
// 坐标系旋转变换（机器人当前朝向角）
static void rotational_coordinates_calc(Coordinate robot_current, Coordinate *robot_error);
// 四个麦克纳姆轮的跟随速度分解
static void chassis_targetFollow_calc(float raw_out[4], Coordinate *robot_error, float pid_out[4], float coeff);
// 四个麦克纳姆轮的旋转速度分解
static void chassis_targetTurn_calc(float raw_out[4], Coordinate *robot_error, float pid_out[4]);
// 自动对准模式的转向速度分解
static void chassis_targetAuto_calc(float raw_out[4], Coordinate *robot_error, float pid_out[4]);
// 综合跟随+旋转的最终轮速+抵偿偏移
static void vPID_Input_calc(float out[4], float follow[4], float turn[4]);

// 自动定位跟随，流水线和参数说明见文件头
void Coordinate_calc_norm(Coordinate target, float vout[4])
{
	
	// 跟随分量 + 旋转分量
	z_calc(Coord.self, target, &Coord.error);
	rotational_coordinates_calc(Coord.self, &Coord.error);
	chassis_targetFollow_calc(s_nav.follow_raw, &Coord.error, s_nav.follow_out, kCoeffFollow);
	chassis_targetTurn_calc(s_nav.turn_raw, &Coord.error, s_nav.turn_out);
	vPID_Input_calc(s_nav.combined_out, s_nav.follow_out, s_nav.turn_out);

	vout[0] = s_nav.combined_out[0];
	vout[1] = s_nav.combined_out[1];
	vout[2] = s_nav.combined_out[2];
	vout[3] = s_nav.combined_out[3];
}

// 自动识别跟随-计算四个麦克纳姆轮的目标速度
void Coordinate_calc_auto(Coordinate target, float vout[4])
{ 
	if(VisData.Kfs_Get_Sig==8)
	{
	 z_calc(Coord.zero, target, &Coord.error);
	 chassis_targetFollow_calc(s_nav.follow_raw, &Coord.error, s_nav.follow_out, kCoeffFollow);
	 chassis_targetAuto_calc(s_nav.turn_raw, &Coord.error, s_nav.auto_out);
	}
	else{
	 z_calc(Coord.zero, target, &Coord.error);
	 rotational_coordinates_calc(Coord.self, &Coord.error);
	 chassis_targetFollow_calc(s_nav.follow_raw, &Coord.error, s_nav.follow_out, kCoeffFollow);
	 chassis_targetAuto_calc(s_nav.turn_raw, &Coord.error, s_nav.auto_out);
  }
	vPID_Input_calc(s_nav.combined_out, s_nav.follow_out, s_nav.auto_out);
	vout[0] = s_nav.combined_out[0];
	vout[1] = s_nav.combined_out[1];
	vout[2] = s_nav.combined_out[2];
	vout[3] = s_nav.combined_out[3];
}

// 垂足算法轨迹跟踪 — 视觉响应慢，目标更新滞后导致轨迹抖动，点跟踪更稳
#if 0
// 获取轨迹函数
void offset_inst_get(Coordinate robot_self, Coordinate *inst)
{
	inst->x = robot_self.x;
	inst->y = robot_self.y;
}

// 抵偿坐标计算：根据机器人当前坐标和轨迹一次函数，计算出一个抵偿坐标，使得机器人能够沿着正确的轨迹前进
static void offset(Coordinate inst, Coordinate self, Coordinate target, Coordinate *offset)
{
	int16_t x0 = self.x;
	int16_t y0 = self.y;

	if (target.y != inst.y)
	{
		// 轨迹方程x=-ky+b
		float k = -(float)(target.x - inst.x) / (target.y - inst.y);
		float b = (float)(target.x * inst.y - inst.x * target.y) / (inst.y - target.y);

		// 垂线方程y-y0=k(x-x0) 算出抵偿坐标目标点
		offset->x = (b + k * k * x0 - k * y0) / (1 + k * k);
		offset->y = (y0 + k * b - k * x0) / (1 + k * k);
	}
	else // 斜率无穷大，轨迹方程y=inst.y
	{
		offset->y = inst.y;
		offset->x = x0;
	}
}
#endif

// 机器人基准坐标系建立（激光雷达→机器人本体）
static void z_calc(Coordinate robot_current, Coordinate robot_target, Coordinate *robot_error)
{
	robot_error->y = robot_current.y - robot_target.y;
	robot_error->x = robot_target.x - robot_current.x;
	robot_error->x = -robot_error->x;
	robot_error->y = -robot_error->y;
	robot_error->angle = -robot_target.angle;
}

// 坐标系旋转变换（跟随当前机器人朝向）
static void rotational_coordinates_calc(Coordinate robot_current, Coordinate *robot_error)
{
	//!!!!!!!!!
	float angle = -(robot_current.angle * kPi / 180.0f);

	int16_t robot_error_x_last = robot_error->x;
	int16_t robot_error_y_last = robot_error->y;

	robot_error->x = cosf(angle) * robot_error_x_last + sinf(angle) * robot_error_y_last;
	robot_error->y = -sinf(angle) * robot_error_x_last + cosf(angle) * robot_error_y_last;
}

// 跟随速度分解：将XY误差映射到四个麦克纳姆轮
static void chassis_targetFollow_calc(float raw_out[4], Coordinate *robot_error, float pid_out[4], float spd_coeff)
{
	float coeff;
	coeff = 0.00065f * expf(kCoeffFollowExp * (abs(robot_error->x) + abs(robot_error->y)));
	coeff = spd_coeff * coeff;
  
	raw_out[0] = -coeff * robot_error->x - coeff * robot_error->y; // FL
	raw_out[1] = -coeff * robot_error->x + coeff * robot_error->y; // FR
	raw_out[2] = coeff * robot_error->x + coeff * robot_error->y;  // BL
	raw_out[3] = coeff * robot_error->x - coeff * robot_error->y;  // BR

	pid_out[0] = PID_Calc(&pid_Follow_1, 0, raw_out[0]);
	pid_out[1] = PID_Calc(&pid_Follow_2, 0, raw_out[1]);
	pid_out[2] = PID_Calc(&pid_Follow_3, 0, raw_out[2]);
	pid_out[3] = PID_Calc(&pid_Follow_4, 0, raw_out[3]);
  
	//增加阻尼，防止超调
	for (int i = 0; i < 4; i++)
	{
		if (pid_out[i] < 0)
		{
			pid_out[i] = pid_out[i] + kDampingFollow; // 向零拉，抑制超调
		}
		else if (pid_out[i] > 0)
		{
			pid_out[i] = pid_out[i] - kDampingFollow;
		}
	}
	//梅林上进行速度限幅
	if ((Coord.target.z == 200 || Coord.target.z == 400 || Coord.target.z == 600) || (Coord.self.z == 200 && Coord.target.z == 0))
	{
		for (int i = 0; i < 4; i++)
			pid_out[i] = fmaxf(-6.0f, fminf(pid_out[i], 6.0f));
	}
	if(VisData.Wp_Trk_Sig==2)
	{
	  for (int i = 0; i < 4; i++)
			pid_out[i] = pid_out[i]*0.35;
	}
	if(VisData.Kfs_Get_Sig==8||VisData.Kfs_Get_Sig==3)
	{
	  for (int i = 0; i < 4; i++)
			pid_out[i] = pid_out[i]*0.5;
	}
}

// 旋转速度分解：角度误差映射到四个轮子
static void chassis_targetTurn_calc(float raw_out[4], Coordinate *robot_error, float pid_out[4])
{
	raw_out[0] = kCoeffTurn * robot_error->angle;
	raw_out[1] = kCoeffTurn * robot_error->angle;
	raw_out[2] = kCoeffTurn * robot_error->angle;
	raw_out[3] = kCoeffTurn * robot_error->angle;
  
	pid_out[0] = PID_Calc(&pid_Turn_1, 0, raw_out[0]);
	pid_out[1] = PID_Calc(&pid_Turn_2, 0, raw_out[1]);
	pid_out[2] = PID_Calc(&pid_Turn_3, 0, raw_out[2]);
	pid_out[3] = PID_Calc(&pid_Turn_4, 0, raw_out[3]);
  
	static float add_extra_max = 90 * 0.020f; 
	static float add_extra = 0.0f;
  
	if (VisData.Kfs_Put_Sig == 0)
	{ 
		// 角度增速：|θ|×0.08 + θ²×0.00015，角度越大推越多，克服静摩擦快转到位
		add_extra = fabsf(robot_error->angle * 0.007f);
		add_extra += robot_error->angle * robot_error->angle * 0.00020f;
    
		if (add_extra > add_extra_max)
		{
			add_extra = add_extra_max;
		}
	} 
	else
	{
		add_extra = 0;
	}
	
	// 向外推输出，放大旋转速度。角度→0 时 add_extra→0，自然交还给 PID 精调
	for (int i = 0; i < 4; i++)
	{
		if (pid_out[i] < 0)
		{
			pid_out[i] = pid_out[i] - add_extra;
		}
		else if (pid_out[i] > 0)
		{
			pid_out[i] = pid_out[i] + add_extra;
		}
	}
//	if ((Coord.target.z == 200 || Coord.target.z == 400 || Coord.target.z == 600) || (Coord.self.z == 200 && Coord.target.z == 0))
//	{
//			for (int i = 0; i < 4; i++)
//		    {pid_out[i] = fmaxf(-6.0f, fminf(pid_out[i], 6.0f));}
//	}
	if(VisData.Kfs_Get_Sig==1)
	{
	    for (int i = 0; i < 4; i++)
		    {pid_out[i] = fmaxf(-5.5f, fminf(pid_out[i], 5.5f));}
	}
//	if(VisData.Wp_Trk_Sig==1&&Grab_count==0)
//	{
//	    for (int i = 0; i < 4; i++)
//		    {pid_out[i] = fmaxf(-3.5f, fminf(pid_out[i], 3.5f));}
//	}
	if(VisData.Kfs_Get_Sig==10||VisData.Kfs_Get_Sig==7)
	{
	    for (int i = 0; i < 4; i++)
		    {pid_out[i] = fmaxf(-15.0f, fminf(pid_out[i], 15.0f));}
	}
}

// 自动对准模式的转向速度分解
static void chassis_targetAuto_calc(float raw_out[4], Coordinate *robot_error, float pid_out[4])
{
	raw_out[0] = kCoeffTurn * robot_error->angle;
	raw_out[1] = kCoeffTurn * robot_error->angle;
	raw_out[2] = kCoeffTurn * robot_error->angle;
	raw_out[3] = kCoeffTurn * robot_error->angle;

	pid_out[0] = PID_Calc(&pid_Auto_1, 0, raw_out[0]);
	pid_out[1] = PID_Calc(&pid_Auto_2, 0, raw_out[1]);
	pid_out[2] = PID_Calc(&pid_Auto_3, 0, raw_out[2]);
	pid_out[3] = PID_Calc(&pid_Auto_4, 0, raw_out[3]);
}

// 最终轮速合成：跟随分量 + 旋转分量
static void vPID_Input_calc(float out[4], float follow[4], float turn[4])
{
	out[0] = follow[0] + turn[0]; // + offset[0];
	out[1] = follow[1] + turn[1]; // + offset[1];
	out[2] = follow[2] + turn[2]; // + offset[2];
	out[3] = follow[3] + turn[3]; // + offset[3];
}





