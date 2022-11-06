#pragma once
/*	关于DLP的简要使用说明 by Lrem
*   1. 程序中的Pattern、Flash、Image、LUT....等名词的含义
*		Pattern -> 图案. 用户投影的每一个图案，图案代表一个最小投影单元
*		Flash   -> 闪存. DLP控制板中的Flash，Flash以24位图像(Image)的形式存储
*		Image   -> 图像. DLP中Flash的存储单元，一张Image具有24位深度，一张Image至多能有24个Pattern(1bit)，最少有3个Pattern(8bit)
*		LUT     -> 查找表. 存储用户想要投影的所有Pattern
*		----------
*		Pattern Bit Depth -> 图案位深度. 每个Pattern的位深度，可以是1-8bit
*		Flash Image -> Flash使用24位Image存储所有的Pattern
*/
#include <Windows.h>
#include <iostream>

#include "dlpc350_common.h"
#include "dlpc350_api.h"
#include "dlpc350_usb.h"

#define MAX_FLASH_IMG_NUM 5			// flash 中最大能存储的图像数量

enum DisplaySatus
{
	DISPLAY_START = 2,
	DISPLAY_PAUSE = 1,
	DISPLAY_STOP = 0,
};

enum TriggerType
{
	INTERNAL_TRIGGER = 0,
	EXTERNAL_POSITIVE = 1,
	EXTERNAL_NEGATIVE = 2,
	NO_INPUT_TRIGGER = 3,
};

enum LEDMode
{
	NO_LED = 0,
	RED = 1,
	GREEN = 2,
	YELLOW = 3,
	BLUE = 4,
	MAGENTA = 5,
	CYAN = 6,
	WHITE = 7,
};

class DLP
{
private:
	std::string name_;
	int width_;
	int heigth_;

	bool isConnected_ = false;			     // 设备连接标识
	bool isAddLUT = false;				     // 是否已将Pattern添加进LUT中
	bool isStart_ = false;					 // 投影开始
	bool isSetPatternSequenceIndex = false;  // 是否已经设置了想要投影的图案序列 (Pattern Sequence)
	bool isUpdatedPatternTime = false;		 // 是否已经更新了曝光时间

	int* flash_index_;						 // Flash中24位图的索引
	int* pattern_index_;					 // Pattern 索引（图案索引）
	int* bit_depth_;						 // 图像位深度
	unsigned char flash_used_index_[MAX_FLASH_IMG_NUM];   // 每次投影序列，需要被使用到的flash图像数量
	int pattern_num_;						 // Pattern数量（图案数量）
	int flash_used_num_ = 0;					 // flash图像数量

	int exposure_time_ = 17000;				 // 默认曝光时间
	int period_time_ = 18000;				 // 默认帧周期，period_time - exposure_time > 230us

public:
	DLP() {}
	DLP(std::string name, int width, int heigth)
		:name_(name), heigth_(heigth), width_(width) {}

	bool ConnectDevice();
	void DisconnectDevice();
	bool ConfigureDevice(bool reset = false);
	bool StartDisplay();
	void StopDisplay();

	void SetPatternTime(int exposure_time, int period_time);
	void SetPatternSequenceIndex(int flash_index[], int pattern_index[], int bit_depth[], int pattern_num);

protected:
	bool AddImgsPatternToLUT();
	bool UpdatePatternTime();
	bool CheckPatternSequence();
	void ClearPatternLUT();
};