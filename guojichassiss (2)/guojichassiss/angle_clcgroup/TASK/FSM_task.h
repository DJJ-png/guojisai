#ifndef FSM_TASK_H
#define FSM_TASK_H
#define mid_rc  200
#define max_rc  660
#define min_rc  -660
typedef enum {
	CHASSISS_PREPARE=0,
	CHASSISS_ENERGEGIVE_RC,
	CHASSISS_AUTOAIM,
	CHASSISS_ENERGYLOSE
}state_chassiss_list;

typedef enum {
	GIMBAL_PREPARE=0,
	GIMBAL_ENERGEGIVE_RC,
	GIMBAL_AUTOAIM,
	GIMBAL_SECONDMODE
}state_gimbal_list;

typedef enum {
	ROAD_STATE_0=0,
	ROAD_STATE_1,//向上走1.06m
	ROAD_STATE_2,//进入阶梯平台
	ROAD_STATE_3,//立桩前
	ROAD_STATE_4,//立桩旁
	ROAD_STATE_5,//到圆盘机
	ROAD_STATE_6,//白点
	ROAD_STATE_7,//阶梯平台
	ROAD_STATE_8,//立仓
	ROAD_STATE_9,//回到起点
	ROAD_STATE_10,//回到起点第二步
	ROAD_STATE_11//静默
}state_road_list;

typedef enum {
	P_STATE_0=0,//到阶梯平台
	P_STATE_1,//校准
	P_STATE_2,//第一个（低）
	P_STATE_3,//第二个（低）
	P_STATE_4,//第一个（高）
	P_STATE_5,//第二个（高）
	P_STATE_6,//第三个（高）
	P_STATE_7,//第四个（高）
	P_STATE_8,//第一个（中）
	P_STATE_9,//第二个（中）
	P_STATE_10,//到达位置
	P_STATE_11//确认拿完
}platform;

typedef enum {
	S_STATE_0=0,//第一列
	S_STATE_1,//第二列
	S_STATE_2,//第三列
	S_STATE_3,//第四列
	S_STATE_4,//
	S_STATE_5,//
	S_STATE_6,//里程计重置&&等待夹爪夹球
}storage;

typedef struct {
	float last_angle;
	float current_angle;
	float change_angle;
}angle_f;

extern angle_f angle_secondmode;
extern state_chassiss_list chassiss_state;
#endif