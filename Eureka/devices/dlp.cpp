#include "dlp.h"

/* 连接相机 */
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

/* 断开相机 */
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

/* 配置投影仪 */
bool DLP::ConfigureDevice(bool reset)
{
	if (!isConnected_)
	{
		std::cout << "The device is not connected" << std::endl;
		return false;
	}

	/* 如果投影仪异常，重置控制器 */
	if (reset == true)
	{
		std::cout << "Reset the controller." << std::endl;
		DLPC350_SoftwareReset();
	}

	/* 关闭低功耗模式. On -> true; Off -> false */
	DLPC350_SetPowerMode(false);

	/* 将DLP设置为PatternDisplay模式. PatternDisplay -> true; VideoDisplay -> false */
	DLPC350_SetMode(true);

	return true;
}

/* 开始投影所选择的图片索引
*
*/
bool DLP::StartDisplay()
{
	if (!isConnected_)
	{
		std::cout << "Couldn't connect the device." << std::endl;
		return false;
	}

	/* 将图片添加进LUT中 */
	if (!AddImgsPatternToLUT())
	{
		std::cout << "Pattern LUT add failed." << std::endl;
		return false;
	}

	/* 如果更新了曝光时间，则重新Send Pattern序列 */
	if (!UpdatePatternTime())
	{
		std::cout << "Pattern time update failed." << std::endl;
		return false;
	}

	/* 检查Pattern Sequence: StartPatLutValidate */
	if (!CheckPatternSequence())
	{
		std::cout << "Start display failed." << std::endl;
		return false;
	}

	/* 开始 Display */
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

	/* 停止 Display */
	std::cout << "Stop Display. " << std::endl;
	DLPC350_PatternDisplay(DISPLAY_STOP);

	isStart_ = false;
}

/* 设置曝光时间和帧周期 */
void DLP::SetPatternTime(int exposure_time, int period_time)
{
	exposure_time_ = exposure_time;
	period_time_ = period_time;
	isUpdatedPatternTime = false;
}

void DLP::SetPatternSequenceIndex(int flash_index[], int pattern_index[], int bit_depth[], int pattern_num)
{
	/* 确保停止投影 */
	StopDisplay();

	/* 清除PatternLUT，以便更新Pattern Sequence Index */
	ClearPatternLUT();

	flash_index_ = flash_index;
	pattern_index_ = pattern_index;
	bit_depth_ = bit_depth;
	pattern_num_ = pattern_num;
	std::cout << "Pattern Num:" << pattern_num_ << std::endl;

	/* 统计此次PatternSequence中使用了多少Flash Image */
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
/* 将需要投影的Pattern(图像)添加到LUT中 */
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
	/* 停止Display */
	StopDisplay();

	/* 在Send LUT之前，必须先设置DisplayMode、Exposure等参数——TiCuide */
	unsigned int numLutEntries = pattern_num_;			                          // 1或者2或者4  必须等于 DEFAULT.SEQPATLUT 中的项目数 = 4
	unsigned int numPatterns = pattern_num_;	                                  // 图案的数量,4，PlayOnce模式下，必须等于 DEFAULT.SEQPATLUT 中的项目数
	unsigned int numImages = flash_used_num_;	                                  // 用作图案(Pattern)的Flash图像数量，确定为2
	DLPC350_SetPatternDisplayMode(false);                                         // 通过Flash读取图像
	DLPC350_SetPatternConfig(numLutEntries, false, numPatterns, numImages);       // 为什么这三个都是一样的呢？ Play Once模式
	DLPC350_SetPatternTriggerMode(1);                                             // Internal trigger
	DLPC350_SetExposure_FramePeriod(exposure_time_, period_time_);                // 设置曝光时间和Pattern周期

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
		if (i == 0)													// 第一张执行缓冲区交换。最后一张，必须关闭DMD，这里我们选择都设置为关闭DMD（关闭DMD == InsertBlack）
		{
			DLPC350_AddToPatLut(INTERNAL_TRIGGER, pattern_index_[i], bit_depth_[i], WHITE, false, true, true, false);
		}
		else if (flash_index_[i] != flash_index_[i - 1])			// 当需要切换Flash Image时，执行缓冲区交换
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

	DLPC350_SetLongAxisImageFlip(false);        // 图像沿长轴翻转：关闭
	DLPC350_SetShortAxisImageFlip(true);        // 图像沿短轴翻转：启动

	isAddLUT = true;							// 添加序列
	isUpdatedPatternTime = true;				// 更新曝光时间

	return true;
}


bool DLP::UpdatePatternTime()
{
	if (isUpdatedPatternTime)
	{
		return true;
	}

	/* 确保先停止投影 */
	StopDisplay();

	std::cout << "Update pattern time." << std::endl;
	/* 更新Pattern Time 重新Send Pattern LUT */
	DLPC350_SetExposure_FramePeriod(exposure_time_, period_time_);                // 设置曝光时间和Pattern周期

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

	DLPC350_SetLongAxisImageFlip(false);        // 图像沿长轴翻转：关闭
	DLPC350_SetShortAxisImageFlip(true);        // 图像沿短轴翻转：启动

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
	for (int i = 0; i < re_check; i++)  // 重复两次检查
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
			Sleep(1000);               // 休眠10ms，重新检查
		}
	}

	return true;
}

/* 清除PatternLUT */
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