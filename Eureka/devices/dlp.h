#pragma once
/*	����DLP�ļ�Ҫʹ��˵�� by Lrem
*   1. �����е�Pattern��Flash��Image��LUT....�����ʵĺ���
*		Pattern -> ͼ��. �û�ͶӰ��ÿһ��ͼ����ͼ������һ����СͶӰ��Ԫ
*		Flash   -> ����. DLP���ư��е�Flash��Flash��24λͼ��(Image)����ʽ�洢
*		Image   -> ͼ��. DLP��Flash�Ĵ洢��Ԫ��һ��Image����24λ��ȣ�һ��Image��������24��Pattern(1bit)��������3��Pattern(8bit)
*		LUT     -> ���ұ�. �洢�û���ҪͶӰ������Pattern
*		----------
*		Pattern Bit Depth -> ͼ��λ���. ÿ��Pattern��λ��ȣ�������1-8bit
*		Flash Image -> Flashʹ��24λImage�洢���е�Pattern
*/
#include <Windows.h>
#include <iostream>

#include "dlpc350_common.h"
#include "dlpc350_api.h"
#include "dlpc350_usb.h"

#define MAX_FLASH_IMG_NUM 5			// flash ������ܴ洢��ͼ������

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

	bool isConnected_ = false;			     // �豸���ӱ�ʶ
	bool isAddLUT = false;				     // �Ƿ��ѽ�Pattern��ӽ�LUT��
	bool isStart_ = false;					 // ͶӰ��ʼ
	bool isSetPatternSequenceIndex = false;  // �Ƿ��Ѿ���������ҪͶӰ��ͼ������ (Pattern Sequence)
	bool isUpdatedPatternTime = false;		 // �Ƿ��Ѿ��������ع�ʱ��

	int* flash_index_;						 // Flash��24λͼ������
	int* pattern_index_;					 // Pattern ������ͼ��������
	int* bit_depth_;						 // ͼ��λ���
	unsigned char flash_used_index_[MAX_FLASH_IMG_NUM];   // ÿ��ͶӰ���У���Ҫ��ʹ�õ���flashͼ������
	int pattern_num_;						 // Pattern������ͼ��������
	int flash_used_num_ = 0;					 // flashͼ������

	int exposure_time_ = 17000;				 // Ĭ���ع�ʱ��
	int period_time_ = 18000;				 // Ĭ��֡���ڣ�period_time - exposure_time > 230us

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