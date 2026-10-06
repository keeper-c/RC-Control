#ifndef BRT_ENCODER_H
#define BRT_ENCODER_H

#include "main.h"

// 编码器数据结构体
typedef struct {
    CAN_HandleTypeDef *hcan;     // 绑定的CAN外设句柄
    uint8_t  node_id;            // 编码器节点ID (出厂默认0x01)
    uint32_t resolution;         // 单圈分辨率 (看编码器外壳标签，如1024/4096/16384等)
    
    uint32_t raw_value;          // 编码器返回的原始32位数值
    int32_t  turns;              // 当前圈数
    float    single_angle;       // 单圈角度 (0 ~ 360.0°)
    float    total_angle;        // 累计总角度
	
    // --- 新增变量：用于无限圈展开 ---
    uint32_t last_raw_value;       // 上一次的原始值
    int32_t  continuous_raw_value; // 拼接后的连续原始值（允许正负无限累加）
    float    continuous_angle;     // 真正的连续角度
    uint8_t  init_flag;            // 初始化标志位	
	
} BRT_Encoder_t;

extern BRT_Encoder_t brt_encoder1,brt_encoder2,brt_encoder3,brt_encoder4;


void BRT_Init(BRT_Encoder_t *enc, CAN_HandleTypeDef *hcan, uint8_t node_id, uint32_t res);
void BRT_ReadPosition(BRT_Encoder_t *enc);
void BRT_SetZero(BRT_Encoder_t *enc);
void BRT_ParseFeedback(BRT_Encoder_t *enc, uint32_t std_id, uint8_t *data);

#endif


