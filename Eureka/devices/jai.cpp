#include "jai.h"

/* 连接相机 */
bool JAI::ConnectDevice()
{
	if (isConnected_)
	{
		cout << "reConnected" << endl;
		return true;
	}

	/* 选择一个设备 */
	if (!FindAndSelectDevice())
	{
		std::cout << "Couldn't find any USB Vision." << std::endl;
		return false;
	}

	cout << "Connected a device " << endl;
	/* 连接到设备 */
	device_ = PvDevice::CreateAndConnect(connectID_, &result_);  // 必须使用CreateAndConnect，而不能直接使用Connect
	if (device_ == NULL)
	{
		std::cout << "Couldn't connect the device." << std::endl;
		return false;
	}

	isConnected_ = true;

	return true;
}

/* 配置相机 
*  trigger_mode: 0-> trigger mode off
*				 1-> trigger mode on
*  pre_acquire_img_num:  [1-MAX_ACQUIRE_NUM] -> Set the number of pictures you want to trigger, in trigger mode
*/
bool JAI::ConfigureDevice(int trigger_mode)
{
	/* 检查设备连接和配置状态 */
	if (!isConnected_)
	{
		std::cout << "The device is not connected，configure failed." << std::endl;
		return false;
	}

	if (isTriggerModeOn_ == trigger_mode)
	{
		std::cout << "reConfigure." << std::endl;
		return true;
	}
	else
	{
		isTriggerModeOn_ = trigger_mode;
	}

	if (isStart_)
	{
		StopAcquireImages();
	}

	if (isexistStreamPipeline_)
	{
		ClearStreamAndPipeline();
	}

	if (isTriggerModeOn_)
	{
		std::cout << "Set device in \"TriggerOn\" mode." << std::endl;
		std::cout << "pre_acquire_img_num_: " << pre_acquire_img_num_ << std::endl;
		ConfigureTriggerOn();
	}
	else
	{
		pre_acquire_img_num_ = 1;
		std::cout << "Set device in \"TriggerOff\" mode." << std::endl;
		std::cout << "pre_acquire_img_num_: " << pre_acquire_img_num_ << std::endl;
		ConfigureTriggerOff();
	}

	return true;
}

/* 开始采集图像 */
bool JAI::StartAcquireImages()
{
	if (!isConnected_)
	{
		std::cout << "The device is not connected，Start acquire failed." << std::endl;
		return false;
	}

	if (isStart_ && isTriggerModeOn_)
	{
		cout << "reStart Acquire " << endl;
		return true;
	}

	if (!isexistStreamPipeline_)	// 如果没有创建Stream，Pipeline，那么现在创建
	{
		if (!CreateStreamAndPipeline())
		{
			return false;
		}
	}

	cout << "Start Acquire " << endl;

	/* 映射GenICam AcquisitionStart和AcquisitionStop命令 */
	PvGenParameterArray* device_params = device_->GetParameters();
	PvGenCommand* start_command = dynamic_cast<PvGenCommand*>(device_params->Get("AcquisitionStart"));

	/* 启动Pipeline. 【此操作，程序会启动一个线程】 */
	pipeline_->Start();

	/* 启用流并发送 acquisitionStart 命令 */
	device_->StreamEnable();
	start_command->Execute();           // Pipeline一启动，图片其实就已经在队列中了

	isStart_ = true;
	return true;
}

/* 停止采集 */
void JAI::StopAcquireImages()
{
	if (!isStart_)
	{
		cout << "reStop Acquire " << endl;
		return;
	}

	cout << "Stop Acquire " << endl;

	/* 映射GenICam AcquisitionStart和AcquisitionStop命令 */
	PvGenParameterArray* device_params = device_->GetParameters();
	PvGenCommand* stop_command = dynamic_cast<PvGenCommand*>(device_params->Get("AcquisitionStop"));

	// Tell the device to stop sending images.
	stop_command->Execute();

	// Disable streaming on the device
	device_->StreamDisable();

	// Stop the pipeline
	pipeline_->Stop();

	isStart_ = false;
}

/* 断开连接 */
void JAI::DisconnectDevice()
{
	if (!isConnected_)
	{
		return;
	}

	std::cout << "Disconnect the device" << std::endl;
	if (isStart_)
	{
		StopAcquireImages();
	}

	if (isexistStreamPipeline_)
	{
		ClearStreamAndPipeline();
	}

	/* 断开设备连接 */
	ClearDevice();

	isConnected_ = false;
}

/* 从获得图像 */
vector_m JAI::GetImgs()
{
	/* 获取新图像前，先清除之前存储的图像 */
	ClearImgs();

	if (!isConnected_)
	{
		std::cout << "The device is not connected，Get image failed." << std::endl;
		return imgs_;
	}

	if (isTriggerModeOn_)
	{
		if (pre_acquire_img_num_ < 1)
		{
			std::cout << "The number of triggered images is not set, please check program" << std::endl; 
		}
		else
		{
			std::cout << "Get imgs by triggering" << std::endl;
			GetImgsFromBuffer();
		}
	}
	else
	{
		
		if (pre_acquire_img_num_ != 1)
		{
			std::cout << "The number of images is not set, please check program" << std::endl;
		}
		else
		{
			std::cout << "Get imgs by user" << std::endl;
			StartAcquireImages();
			GetImgsFromBuffer();
			StopAcquireImages();
		}
	}

	return imgs_; 
}


void JAI::SetPreAcquireImgNum(int pre_acquire_img_num)
{
	if (!isTriggerModeOn_)		// 非触发模式下，预拍摄的图像数量为1
	{
		pre_acquire_img_num_ = 1;
	}
	else
	{
		pre_acquire_img_num_ = pre_acquire_img_num;
	}
}



/*------------Protected Function--------------*/ 
/* 通过PvSystem查找一个可用的USB接口或设备 */
bool JAI::FindAndSelectDevice()
{
	PvSystem aPvSystem;
	uint32_t aTimeout = 1500;
	aPvSystem.SetDetectionTimeout(aTimeout);
	aPvSystem.Find();
	if (aPvSystem.GetDeviceCount() == 0)
	{
		return false;
	}
	else
	{
		int device_index = 0;  // 默认选择第一个设备
		const PvDeviceInfo* aDeviceInfo_ = aPvSystem.GetDeviceInfo(device_index);
		std::cout << "Find a device:" << aDeviceInfo_->GetDisplayID().GetAscii() << std::endl;
		connectID_ = aDeviceInfo_->GetConnectionID();
		return true;
	}
}

/* 创建并初始化Stream和Pipeline，如果已经创建无需重复创建 */
bool JAI::CreateStreamAndPipeline()
{
	if (isexistStreamPipeline_)
	{
		return true;
	}

	/* 创建Stream */
	stream_ = PvStream::CreateAndOpen(connectID_, &result_);
	if (!stream_->IsOpen())
	{
		std::cout << "Stream Creation failed." << std::endl;
		return false;
	}
	/*  创建Pipeline，并分配缓冲区.*/
	pipeline_ = new PvPipeline(stream_);
	if (pipeline_ != NULL)
	{
		// Reading payload size from device
		uint32_t size = device_->GetPayloadSize();

		// Set the Buffer count and the Buffer size
		pipeline_->SetBufferCount(BUFFER_COUNT);
		pipeline_->SetBufferSize(size);
	}
	isexistStreamPipeline_ = true;
	return true;
}


/* 开启外触发模式，被动拍照 */
void JAI::ConfigureTriggerOn()
{
	PvGenParameterArray* device_params = device_->GetParameters();
	PvGenEnum* parameEnum;
	parameEnum = device_params->GetEnum("AcquisitionMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(2); }	 // Continuous 2

	/* 优先设置ExposureMode —— 曝光模式 */
	parameEnum = device_params->GetEnum("ExposureMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(2); }   // Off 0; timed 2; Trigger Width 2


	/* 设置外触发 */
	parameEnum = device_params->GetEnum("TriggerSelector");		// TriggerSelector 必须设置为FrameStart类型，才能自然切换外触发模式和单拍模式
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(0); }	 // AcquisitionStart 0; AcquisitionStart 1; FrameStart 2; Acquisition Transfer Start 3

	parameEnum = device_params->GetEnum("TriggerMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(0); }	// TriggerMode Off 0; TriggerMode On 1; 

	parameEnum = device_params->GetEnum("TriggerSelector");		// TriggerSelector 必须设置为FrameStart类型，才能自然切换外触发模式和单拍模式
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(2); }	 // AcquisitionStart 0; AcquisitionStart 1; FrameStart 2; Acquisition Transfer Start 3

	parameEnum = device_params->GetEnum("TriggerMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(1); }	// TriggerMode On 1

	parameEnum = device_params->GetEnum("TriggerSource");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(13); }  // Line5 13，Line5=ExternalTriggerSignal

	parameEnum = device_params->GetEnum("TriggerActivation");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(2); }   // RisingEdge 0; FalingEdge 1; Level High 2; Level Low 3;    


}


/* 关闭外触发模式，主动拍照 */
void JAI::ConfigureTriggerOff()
{
	PvGenParameterArray* device_params = device_->GetParameters();
	PvGenEnum* parameEnum;
	parameEnum = device_params->GetEnum("AcquisitionMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(0); }	 // SingleFrame 0

	/* 先把ExposureMode设为timed */
	parameEnum = device_params->GetEnum("ExposureMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(1); }   // Off 0; timed 2; Trigger Width 2

	/* 然后关闭外触发模式 */
	parameEnum = device_params->GetEnum("TriggerSelector");		// TriggerSelector 必须设置为FrameStart类型，才能自然切换外触发模式和单拍模式
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(2); }	 // AcquisitionStart 0; AcquisitionStart 1; FrameStart 2; Acquisition Transfer Start 3
	parameEnum = device_params->GetEnum("TriggerMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(0); }	// TriggerMode Off 0; TriggerMode Off 1; 
}

/* 清除Stream和Pipeline */
void JAI::ClearStreamAndPipeline()
{
	if (!isexistStreamPipeline_)
	{
		return;
	}

	/* 清除Pipeline */
	delete pipeline_;
	/* 清除stream */
	stream_->Close();
	PvStream::Free(stream_);

	/* 清除已采集图像计数 */
	acquire_img_all_ = 0;
	isexistStreamPipeline_ = false;
}

/* 从缓冲区中获取图像 */
void JAI::GetImgsFromBuffer()
{
	/* 设置显示字符 */
	char lDoodle[] = "|\\-|-/";
	int lDoodleIndex = 0;
	double lFrameRateVal = 0.0;
	double lBandwidthVal = 0.0;

	/* 获取图像，直到获取到指定数量的图片，或用户指示停止 */
	uint32_t buffer_time_out = 1000; // MAX = 0xFFFFFFFF = 4294967295，每隔一段时间去扫描是否缓冲区存在图像

	//clock_t jai_start = clock();
	while (!PvKbHit())
	{
		PvBuffer* lBuffer = NULL;		// 指向一个有效缓冲区的指针
		PvResult lOperationResult;

		/* 检索下一个缓冲区 */
		PvResult lResult = pipeline_->RetrieveNextBuffer(&lBuffer,buffer_time_out, &lOperationResult);
		if (lResult.IsOK())
		{
			if (lOperationResult.IsOK())
			{
				PvPayloadType lType;	// 有效负载类型：图像、原始数据、File……
				lType = lBuffer->GetPayloadType();

				if (lType == PvPayloadTypeImage)		// 如果有效数据类型为图像
				{

					// If the buffer contains an image, display width and height.
					uint32_t lWidth = 0, lHeight = 0;
					// Get image specific buffer interface.
					PvImage* lImage = lBuffer->GetImage();

					// Read width, height.
					lWidth = lImage->GetWidth();
					lHeight = lImage->GetHeight();

					// 保存为Mat
					unsigned char* data = lImage->GetDataPointer();
					cv::Mat aMat(1, lWidth * lHeight, CV_8UC1);
					aMat.forEach<uchar>([&data](uchar& val, const int* pos) {
						val = data[pos[1]];
						});
					aMat = aMat.reshape(0, lHeight);
					imgs_.push_back(aMat);
					acquire_img_all_++;
					// 如果得到相应数量的图像，则停止采集
					if (imgs_.size() == pre_acquire_img_num_) { break; }
				}
				else {
					cout << " This data isn't a image ";
				}
			}
			else
			{
				// Non OK operational result
				cout << lDoodle[lDoodleIndex] << " " << lOperationResult.GetCodeString().GetAscii() << "\r";
			}
			// 将缓冲区释放回管道
			pipeline_->ReleaseBuffer(lBuffer); 
		}
		else
		{
			// Time Out，退出
			cout << lDoodle[lDoodleIndex] << " " << lResult.GetCodeString().GetAscii() << "\r";
			break;
		}
		++lDoodleIndex %= 6;
	}

	/* 如果拍的所有图，快要超出缓冲区大小，重新分配缓冲区 */
	if (acquire_img_all_ >= MAX_ACQUIRE_NUM - pre_acquire_img_num_)
	{
		uint32_t size = device_->GetPayloadSize();
		pipeline_->SetBufferCount(BUFFER_COUNT);
		pipeline_->SetBufferSize(size);
		acquire_img_all_ = 0;
	}
}

/* 清除图像 */
void JAI::ClearImgs() { imgs_.clear(); }

/* 断开并清除Device */
void JAI::ClearDevice()
{
	if (isConnected_)
	{
		device_->Disconnect();
		PvDevice::Free(device_);
	}
}
