#ifndef LUENBERGER_H
#define LUENBERGER_H

typedef struct{//预测位置，速度（需要用到加速度（但我不想用，因为gyro噪声太大了））
	float p_k_1;
	float p_k;
	float p_k_correct;//融合的修正值
	float v_k_1;
	float v_k;
	float v_k_correct;//同理
}Luenberger_data;

typedef struct{
	float l1;
	float l2
}L_t;

typedef struct{
	float cnt_c;
	float cnt_p;
	float cnt_delta;
	float cnt_realdelta;
	float angle_delta;
	float lenth_delta;
	float p;
	float wheel_lenth;
}position_ecd;

#endif