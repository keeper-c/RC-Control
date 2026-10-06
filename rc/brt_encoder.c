#include "brt_encoder.h"
#include "Can_receive.h"

BRT_Encoder_t brt_encoder1, brt_encoder2, brt_encoder3, brt_encoder4;

// 初始化编码器
void BRT_Init(BRT_Encoder_t *enc, CAN_HandleTypeDef *hcan, uint8_t node_id, uint32_t res) {
	enc->hcan = hcan;
	enc->node_id = node_id;
	enc->resolution = res;

	enc->raw_value = 0;
	enc->turns = 0;
	enc->single_angle = 0.0f;
	enc->total_angle = 0.0f;

	// 初始化新增变量
	enc->last_raw_value = 0;
	enc->continuous_raw_value = 0;
	enc->continuous_angle = 0.0f;
	enc->init_flag = 0;
}

// 查询当前位置指令
void BRT_ReadPosition(BRT_Encoder_t *enc) {
	uint8_t txData[4];
	txData[0] = 0x04;          // 数据长度
	txData[1] = enc->node_id;  // 设备ID
	txData[2] = 0x01;          // 指令: 0x01 读取编码器值
	txData[3] = 0x00;          // 补充数据

	// 调用你已有的发送函数，发4字节
	CAN_TransmitMessage_StdID(enc->hcan, enc->node_id, txData, 4);
}

// 设置当前位置为零点指令
void BRT_SetZero(BRT_Encoder_t *enc) {
	uint8_t txData[4];
	txData[0] = 0x04;
	txData[1] = enc->node_id;
	txData[2] = 0x06;          // 指令: 0x06 设置零点
	txData[3] = 0x00;

	CAN_TransmitMessage_StdID(enc->hcan, enc->node_id, txData, 4);
}

// 解析接收到的CAN数据 (在接收中断中调用)
void BRT_ParseFeedback(BRT_Encoder_t *enc, uint32_t std_id, uint8_t *data) {
	// 说明书6.3.3：编码器回传时的ID等于其自身ID，并且数据第二位等于其自身ID
	if (std_id == enc->node_id) {
		uint8_t cmd = data[2]; // 获取指令码

		if (cmd == 0x01) { // 如果是读取位置的返回数据
			// 说明书6.5.3: 返回32位无符号整数，低字节在前(小端模式)
			enc->raw_value = (uint32_t)data[3] |
			                 ((uint32_t)data[4] << 8) |
			                 ((uint32_t)data[5] << 16) |
			                 ((uint32_t)data[6] << 24);

// ================= 新增：无限圈展开逻辑 =================
			if (enc->init_flag == 0) {
				enc->last_raw_value = enc->raw_value;
				enc->continuous_raw_value = enc->raw_value;
				enc->init_flag = 1;
			} else {
				int32_t delta = (int32_t)enc->raw_value - (int32_t)enc->last_raw_value;

				// 16圈编码器的最大量程 = 16 * 单圈分辨率
				int32_t full_range = 16 * enc->resolution;
				int32_t half_range = full_range / 2;

				// 处理越界跳变 (就近累加)
				if (delta > half_range) {
					delta -= full_range; // 例如 16383突变到0，delta为负的16383，修正为 +1
				} else if (delta < -half_range) {
					delta += full_range; // 例如 0突变到16383，delta为正的16383，修正为 -1
				}

				enc->continuous_raw_value += delta;
				enc->last_raw_value = enc->raw_value;
			}
			// ========================================================

			// 计算圈数: 原始值 / 单圈分辨率
			enc->turns = enc->raw_value / enc->resolution;

			// 计算单圈内的当前角度 (0~360度)
			uint32_t remainder = enc->raw_value % enc->resolution;
			enc->single_angle = (float)remainder * 360.0f / (float)enc->resolution;

			// 计算总角度
			enc->total_angle = (float)enc->raw_value * 360.0f / (float)enc->resolution;

			//// 计算展开后的绝对连续角度
			enc->continuous_angle = (float)enc->continuous_raw_value * 360.0f / (float)enc->resolution;

		}
	}
}
