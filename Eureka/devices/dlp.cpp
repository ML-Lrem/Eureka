#include "dlp.h"

/* ������� */
bool DLP::ConnectDevice()
{
	if (isConnected_)
	{
		std::cout << "reConnect." << std::endl;
		return true;
	}

	DLPC350_USB_Init();
	if (DLPC350_USB_Open() != 0)	// if Open return 0; else return -1
	{
		std::cout << "Couldn't connect the device." << std::endl;
		return false;
	}

	std::cout << "DLP Connected." << std::endl;

	isConnected_ = true;
	return true;
}

/* �Ͽ���� */
void DLP::DisconnectDevice()
{
	if (!isConnected_)
	{
		std::cout << "reDisconnect." << std::endl;
		return;
	}

	StopDisplay();
	ClearPatternLUT();

	std::cout << "DLP Disconnected." << std::endl;
	DLPC350_USB_Close();
}

/* ����ͶӰ�� */
bool DLP::ConfigureDevice(bool reset)
{
	if (!isConnected_)
	{
		std::cout << "The device is not connected" << std::endl;
		return false;
	}

	/* ���ͶӰ���쳣�����ÿ����� */
	if (reset == true)
	{
		std::cout << "Reset the controller." << std::endl;
		DLPC350_SoftwareReset();
	}

	/* �رյ͹���ģʽ. On -> true; Off -> false */
	DLPC350_SetPowerMode(false);

	/* ��DLP����ΪPatternDisplayģʽ. PatternDisplay -> true; VideoDisplay -> false */
	DLPC350_SetMode(true);

	return true;
}

/* ��ʼͶӰ��ѡ���ͼƬ����
*
*/
bool DLP::StartDisplay()
{
	if (!isConnected_)
	{
		std::cout << "Couldn't connect the device." << std::endl;
		return false;
	}

	/* ��ͼƬ��ӽ�LUT�� */
	if (!AddImgsPatternToLUT())
	{
		std::cout << "Pattern LUT add failed." << std::endl;
		return false;
	}

	/* ����������ع�ʱ�䣬������Send Pattern���� */
	if (!UpdatePatternTime())
	{
		std::cout << "Pattern time update failed." << std::endl;
		return false;
	}

	/* ���Pattern Sequence: StartPatLutValidate */
	if (!CheckPatternSequence())
	{
		std::cout << "Start display failed." << std::endl;
		return false;
	}

	/* ��ʼ Display */
	std::cout << "Start display." << std::endl;
	DLPC350_PatternDisplay(DISPLAY_START);

	isStart_ = true;
	return true;
}

void DLP::StopDisplay()
{
	if (!isStart_)
	{
		return;
	}

	/* ֹͣ Display */
	std::cout << "Stop Display. " << std::endl;
	DLPC350_PatternDisplay(DISPLAY_STOP);

	isStart_ = false;
}

/* �����ع�ʱ���֡���� */
void DLP::SetPatternTime(int exposure_time, int period_time)
{
	exposure_time_ = exposure_time;
	period_time_ = period_time;
	isUpdatedPatternTime = false;
}

void DLP::SetPatternSequenceIndex(int flash_index[], int pattern_index[], int bit_depth[], int pattern_num)
{
	/* ȷ��ֹͣͶӰ */
	StopDisplay();

	/* ���PatternLUT���Ա����Pattern Sequence Index */
	ClearPatternLUT();

	flash_index_ = flash_index;
	pattern_index_ = pattern_index;
	bit_depth_ = bit_depth;
	pattern_num_ = pattern_num;
	std::cout << "Pattern Num:" << pattern_num_ << std::endl;

	/* ͳ�ƴ˴�PatternSequence��ʹ���˶���Flash Image */
	for (int i = 0; i < pattern_num_; i++)
	{
		if (flash_index_[i] != flash_index_[i + 1])
		{
			flash_used_index_[flash_used_num_] = flash_index_[i];
			flash_used_num_++;
		}
	}

	isSetPatternSequenceIndex = true;
}


/*-------------- protected --------------*/
/* ����ҪͶӰ��Pattern(ͼ��)��ӵ�LUT�� */
bool DLP::AddImgsPatternToLUT()
{
	if (isAddLUT)
	{
		std::cout << "Pattern LUT is added." << std::endl;
		return true;
	}

	if (!isSetPatternSequenceIndex)
	{
		std::cout << "No Pattern Sequence Index is set." << std::endl;
		return false;
	}

	std::cout << "Add Pattern to LUT." << std::endl;
	/* ֹͣDisplay */
	StopDisplay();

	/* ��Send LUT֮ǰ������������DisplayMode��Exposure�Ȳ�������TiCuide */
	unsigned int numLutEntries = pattern_num_;			                          // 1����2����4  ������� DEFAULT.SEQPATLUT �е���Ŀ�� = 4
	unsigned int numPatterns = pattern_num_;	                                  // ͼ��������,4��PlayOnceģʽ�£�������� DEFAULT.SEQPATLUT �е���Ŀ��
	unsigned int numImages = flash_used_num_;	                                  // ����ͼ��(Pattern)��Flashͼ��������ȷ��Ϊ2
	DLPC350_SetPatternDisplayMode(false);                                         // ͨ��Flash��ȡͼ��
	DLPC350_SetPatternConfig(numLutEntries, false, numPatterns, numImages);       // Ϊʲô����������һ�����أ� Play Onceģʽ
	DLPC350_SetPatternTriggerMode(1);                                             // Internal trigger
	DLPC350_SetExposure_FramePeriod(exposure_time_, period_time_);                // �����ع�ʱ���Pattern����

	/* Set up the image indexes if using images from flash. \ Send Image and Pattern */
	unsigned int numEntries = flash_used_num_;
	if (DLPC350_SendImageLut(flash_used_index_, numEntries) < 0)
	{
		std::cout << "Error Sending Image LUT" << std::endl;
		return false;
	}

	/* Add Pattern to LUT */
	for (int i = 0; i < pattern_num_; i++)
	{
		if (i == 0)													// ��һ��ִ�л��������������һ�ţ�����ر�DMD����������ѡ������Ϊ�ر�DMD���ر�DMD == InsertBlack��
		{
			DLPC350_AddToPatLut(INTERNAL_TRIGGER, pattern_index_[i], bit_depth_[i], WHITE, false, true, true, false);
		}
		else if (flash_index_[i] != flash_index_[i - 1])			// ����Ҫ�л�Flash Imageʱ��ִ�л���������
		{
			DLPC350_AddToPatLut(INTERNAL_TRIGGER, pattern_index_[i], bit_depth_[i], WHITE, false, true, true, false);
		}
		else
		{
			DLPC350_AddToPatLut(INTERNAL_TRIGGER, pattern_index_[i], bit_depth_[i], WHITE, false, true, false, false);
		}
	}
	if (DLPC350_SendPatLut() < 0)
	{
		std::cout << "Error Sending Pattern LUT" << std::endl;
		return false;
	}

	DLPC350_SetLongAxisImageFlip(false);        // ͼ���س��ᷭת���ر�
	DLPC350_SetShortAxisImageFlip(true);        // ͼ���ض��ᷭת������

	isAddLUT = true;							// �������
	isUpdatedPatternTime = true;				// �����ع�ʱ��

	return true;
}


bool DLP::UpdatePatternTime()
{
	if (isUpdatedPatternTime)
	{
		return true;
	}

	/* ȷ����ֹͣͶӰ */
	StopDisplay();

	std::cout << "Update pattern time." << std::endl;
	/* ����Pattern Time ����Send Pattern LUT */
	DLPC350_SetExposure_FramePeriod(exposure_time_, period_time_);                // �����ع�ʱ���Pattern����

	/* Send Image and Pattern */
	unsigned int numEntries = flash_used_num_;
	if (DLPC350_SendImageLut(flash_used_index_, numEntries) < 0)
	{
		std::cout << "Error Sending Image LUT" << std::endl;
		return false;
	}
	if (DLPC350_SendPatLut() < 0)
	{
		std::cout << "Error Sending Pattern LUT" << std::endl;
		return false;
	}

	DLPC350_SetLongAxisImageFlip(false);        // ͼ���س��ᷭת���ر�
	DLPC350_SetShortAxisImageFlip(true);        // ͼ���ض��ᷭת������

	isUpdatedPatternTime = true;

	return true;
}


bool DLP::CheckPatternSequence()
{
	if (isStart_)
	{
		//std::cout << "Projector is runing, no need to Validate." << std::endl;
		return true;
	}

	unsigned int status;
	bool ready;

	if (DLPC350_StartPatLutValidate() < 0)
	{
		std::cout << "error validating LUT data" << std::endl;
		return false;
	}

	int re_check = 5;
	for (int i = 0; i < re_check; i++)  // �ظ����μ��
	{
		if (DLPC350_CheckPatLutValidate(&ready, &status) < 0)
		{
			std::cout << "error validating LUT data" << std::endl;
			return false;
		}
		if (ready)
		{
			std::cout << "Is Ready." << std::endl;
			break;
		}
		else
		{
			if (i == re_check - 1)
			{
				return false;
			}
			Sleep(1000);               // ����10ms�����¼��
		}
	}

	return true;
}

/* ���PatternLUT */
void DLP::ClearPatternLUT()
{
	if (!isAddLUT)
	{
		return;
	}

	std::cout << "Clear Pattern LUT." << std::endl;
	DLPC350_ClearPatLut();

	isAddLUT = false;
}