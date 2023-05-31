/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2023-03-02

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2023-03-02

All rights reserved
***********************************************/

#ifndef __LIDAR_H
#define	__LIDAR_H

#include "Header.h"




#include "Header.h"


//可选雷达
//目前只有LD14
#define 	LD14		
//#define 	LD06
//#define 	N10
//#define 	M10


#ifdef LD14

#define ANGLE_PER_FRAME 				12
#define HEADER 							0x54
#define POINT_PER_PACK 					12
#define LENGTH  						0x2C 	//低五位是一帧数据接收到的点数，目前固定是12，高三位预留


#define offset_x						5.9f
#define offset_y						-20.14f

#endif


typedef struct __attribute__((packed)) Point_Data
{
	uint16_t distance;//距离
	uint8_t confidence;//置信度
	
}LidarPointStructDef;//一帧数据中每个点包含的数据


typedef struct __attribute__((packed)) Pack_Data
{
	uint8_t header;
	uint8_t ver_len;
	uint16_t speed;
	uint16_t start_angle;
	LidarPointStructDef point[POINT_PER_PACK];
	uint16_t end_angle;
	uint16_t timestamp;
	uint8_t crc8;
}LiDARFrameTypeDef;//一帧数据结构体

typedef struct __attribute__((packed)) PointDataProcess_
{
	uint16_t distance;
	float angle;
	float x;
	float y;
}PointDataProcessDef;//经过处理后的数据



extern PointDataProcessDef PointDataProcess[420];//更新390个数据
extern PointDataProcessDef Dataprocess[400];      //用于小车避障、跟随、走直线、ELE雷达避障的雷达数据
extern PointDataProcessDef TempData[12];  //超过了0度的下一圈数据临时存储
extern LiDARFrameTypeDef Pack_Data;			//雷达接收的数据储存在这个变量之中	


void data_process(void);


#endif


