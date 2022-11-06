#include "jai.h"

/* ������� */
bool JAI::ConnectDevice()
{
	if (isConnected_)
	{
		cout << "reConnected" << endl;
		return true;
	}

	/* ѡ��һ���豸 */
	if (!FindAndSelectDevice())
	{
		std::cout << "Couldn't find any USB Vision." << std::endl;
		return false;
	}

	cout << "Connected a device " << endl;
	/* ���ӵ��豸 */
	device_ = PvDevice::CreateAndConnect(connectID_, &result_);  // ����ʹ��CreateAndConnect��������ֱ��ʹ��Connect
	if (device_ == NULL)
	{
		std::cout << "Couldn't connect the device." << std::endl;
		return false;
	}

	isConnected_ = true;

	return true;
}

/* ������� 
*  trigger_mode: 0-> trigger mode off
*				 1-> trigger mode on
*  pre_acquire_img_num:  [1-MAX_ACQUIRE_NUM] -> Set the number of pictures you want to trigger, in trigger mode
*/
bool JAI::ConfigureDevice(int trigger_mode)
{
	/* ����豸���Ӻ�����״̬ */
	if (!isConnected_)
	{
		std::cout << "The device is not connected��configure failed." << std::endl;
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

/* ��ʼ�ɼ�ͼ�� */
bool JAI::StartAcquireImages()
{
	if (!isConnected_)
	{
		std::cout << "The device is not connected��Start acquire failed." << std::endl;
		return false;
	}

	if (isStart_ && isTriggerModeOn_)
	{
		cout << "reStart Acquire " << endl;
		return true;
	}

	if (!isexistStreamPipeline_)	// ���û�д���Stream��Pipeline����ô���ڴ���
	{
		if (!CreateStreamAndPipeline())
		{
			return false;
		}
	}

	cout << "Start Acquire " << endl;

	/* ӳ��GenICam AcquisitionStart��AcquisitionStop���� */
	PvGenParameterArray* device_params = device_->GetParameters();
	PvGenCommand* start_command = dynamic_cast<PvGenCommand*>(device_params->Get("AcquisitionStart"));

	/* ����Pipeline. ���˲��������������һ���̡߳� */
	pipeline_->Start();

	/* ������������ acquisitionStart ���� */
	device_->StreamEnable();
	start_command->Execute();           // Pipelineһ������ͼƬ��ʵ���Ѿ��ڶ�������

	isStart_ = true;
	return true;
}

/* ֹͣ�ɼ� */
void JAI::StopAcquireImages()
{
	if (!isStart_)
	{
		cout << "reStop Acquire " << endl;
		return;
	}

	cout << "Stop Acquire " << endl;

	/* ӳ��GenICam AcquisitionStart��AcquisitionStop���� */
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

/* �Ͽ����� */
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

	/* �Ͽ��豸���� */
	ClearDevice();

	isConnected_ = false;
}

/* �ӻ��ͼ�� */
vector_m JAI::GetImgs()
{
	/* ��ȡ��ͼ��ǰ�������֮ǰ�洢��ͼ�� */
	ClearImgs();

	if (!isConnected_)
	{
		std::cout << "The device is not connected��Get image failed." << std::endl;
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
	if (!isTriggerModeOn_)		// �Ǵ���ģʽ�£�Ԥ�����ͼ������Ϊ1
	{
		pre_acquire_img_num_ = 1;
	}
	else
	{
		pre_acquire_img_num_ = pre_acquire_img_num;
	}
}



/*------------Protected Function--------------*/ 
/* ͨ��PvSystem����һ�����õ�USB�ӿڻ��豸 */
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
		int device_index = 0;  // Ĭ��ѡ���һ���豸
		const PvDeviceInfo* aDeviceInfo_ = aPvSystem.GetDeviceInfo(device_index);
		std::cout << "Find a device:" << aDeviceInfo_->GetDisplayID().GetAscii() << std::endl;
		connectID_ = aDeviceInfo_->GetConnectionID();
		return true;
	}
}

/* ��������ʼ��Stream��Pipeline������Ѿ����������ظ����� */
bool JAI::CreateStreamAndPipeline()
{
	if (isexistStreamPipeline_)
	{
		return true;
	}

	/* ����Stream */
	stream_ = PvStream::CreateAndOpen(connectID_, &result_);
	if (!stream_->IsOpen())
	{
		std::cout << "Stream Creation failed." << std::endl;
		return false;
	}
	/*  ����Pipeline�������仺����.*/
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


/* �����ⴥ��ģʽ���������� */
void JAI::ConfigureTriggerOn()
{
	PvGenParameterArray* device_params = device_->GetParameters();
	PvGenEnum* parameEnum;
	parameEnum = device_params->GetEnum("AcquisitionMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(2); }	 // Continuous 2

	/* ��������ExposureMode ���� �ع�ģʽ */
	parameEnum = device_params->GetEnum("ExposureMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(2); }   // Off 0; timed 2; Trigger Width 2


	/* �����ⴥ�� */
	parameEnum = device_params->GetEnum("TriggerSelector");		// TriggerSelector ��������ΪFrameStart���ͣ�������Ȼ�л��ⴥ��ģʽ�͵���ģʽ
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(0); }	 // AcquisitionStart 0; AcquisitionStart 1; FrameStart 2; Acquisition Transfer Start 3

	parameEnum = device_params->GetEnum("TriggerMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(0); }	// TriggerMode Off 0; TriggerMode On 1; 

	parameEnum = device_params->GetEnum("TriggerSelector");		// TriggerSelector ��������ΪFrameStart���ͣ�������Ȼ�л��ⴥ��ģʽ�͵���ģʽ
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(2); }	 // AcquisitionStart 0; AcquisitionStart 1; FrameStart 2; Acquisition Transfer Start 3

	parameEnum = device_params->GetEnum("TriggerMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(1); }	// TriggerMode On 1

	parameEnum = device_params->GetEnum("TriggerSource");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(13); }  // Line5 13��Line5=ExternalTriggerSignal

	parameEnum = device_params->GetEnum("TriggerActivation");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(2); }   // RisingEdge 0; FalingEdge 1; Level High 2; Level Low 3;    


}


/* �ر��ⴥ��ģʽ���������� */
void JAI::ConfigureTriggerOff()
{
	PvGenParameterArray* device_params = device_->GetParameters();
	PvGenEnum* parameEnum;
	parameEnum = device_params->GetEnum("AcquisitionMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(0); }	 // SingleFrame 0

	/* �Ȱ�ExposureMode��Ϊtimed */
	parameEnum = device_params->GetEnum("ExposureMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(1); }   // Off 0; timed 2; Trigger Width 2

	/* Ȼ��ر��ⴥ��ģʽ */
	parameEnum = device_params->GetEnum("TriggerSelector");		// TriggerSelector ��������ΪFrameStart���ͣ�������Ȼ�л��ⴥ��ģʽ�͵���ģʽ
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(2); }	 // AcquisitionStart 0; AcquisitionStart 1; FrameStart 2; Acquisition Transfer Start 3
	parameEnum = device_params->GetEnum("TriggerMode");
	if (parameEnum == NULL) { std::cout << "This parameter does not exist in the device." << std::endl; }
	else { parameEnum->SetValue(0); }	// TriggerMode Off 0; TriggerMode Off 1; 
}

/* ���Stream��Pipeline */
void JAI::ClearStreamAndPipeline()
{
	if (!isexistStreamPipeline_)
	{
		return;
	}

	/* ���Pipeline */
	delete pipeline_;
	/* ���stream */
	stream_->Close();
	PvStream::Free(stream_);

	/* ����Ѳɼ�ͼ����� */
	acquire_img_all_ = 0;
	isexistStreamPipeline_ = false;
}

/* �ӻ������л�ȡͼ�� */
void JAI::GetImgsFromBuffer()
{
	/* ������ʾ�ַ� */
	char lDoodle[] = "|\\-|-/";
	int lDoodleIndex = 0;
	double lFrameRateVal = 0.0;
	double lBandwidthVal = 0.0;

	/* ��ȡͼ��ֱ����ȡ��ָ��������ͼƬ�����û�ָʾֹͣ */
	uint32_t buffer_time_out = 1000; // MAX = 0xFFFFFFFF = 4294967295��ÿ��һ��ʱ��ȥɨ���Ƿ񻺳�������ͼ��

	//clock_t jai_start = clock();
	while (!PvKbHit())
	{
		PvBuffer* lBuffer = NULL;		// ָ��һ����Ч��������ָ��
		PvResult lOperationResult;

		/* ������һ�������� */
		PvResult lResult = pipeline_->RetrieveNextBuffer(&lBuffer,buffer_time_out, &lOperationResult);
		if (lResult.IsOK())
		{
			if (lOperationResult.IsOK())
			{
				PvPayloadType lType;	// ��Ч�������ͣ�ͼ��ԭʼ���ݡ�File����
				lType = lBuffer->GetPayloadType();

				if (lType == PvPayloadTypeImage)		// �����Ч��������Ϊͼ��
				{

					// If the buffer contains an image, display width and height.
					uint32_t lWidth = 0, lHeight = 0;
					// Get image specific buffer interface.
					PvImage* lImage = lBuffer->GetImage();

					// Read width, height.
					lWidth = lImage->GetWidth();
					lHeight = lImage->GetHeight();

					// ����ΪMat
					unsigned char* data = lImage->GetDataPointer();
					cv::Mat aMat(1, lWidth * lHeight, CV_8UC1);
					aMat.forEach<uchar>([&data](uchar& val, const int* pos) {
						val = data[pos[1]];
						});
					aMat = aMat.reshape(0, lHeight);
					imgs_.push_back(aMat);
					acquire_img_all_++;
					// ����õ���Ӧ������ͼ����ֹͣ�ɼ�
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
			// ���������ͷŻعܵ�
			pipeline_->ReleaseBuffer(lBuffer); 
		}
		else
		{
			// Time Out���˳�
			cout << lDoodle[lDoodleIndex] << " " << lResult.GetCodeString().GetAscii() << "\r";
			break;
		}
		++lDoodleIndex %= 6;
	}

	/* ����ĵ�����ͼ����Ҫ������������С�����·��仺���� */
	if (acquire_img_all_ >= MAX_ACQUIRE_NUM - pre_acquire_img_num_)
	{
		uint32_t size = device_->GetPayloadSize();
		pipeline_->SetBufferCount(BUFFER_COUNT);
		pipeline_->SetBufferSize(size);
		acquire_img_all_ = 0;
	}
}

/* ���ͼ�� */
void JAI::ClearImgs() { imgs_.clear(); }

/* �Ͽ������Device */
void JAI::ClearDevice()
{
	if (isConnected_)
	{
		device_->Disconnect();
		PvDevice::Free(device_);
	}
}
