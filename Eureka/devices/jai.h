#pragma once
#include <windows.h>				// ����jai �ж�ƽ̨��������ֲ��
#include <iostream>
#include <opencv.hpp>
#include <vector>

#include <PvSampleUtils.h>
#include <PvSystem.h>
#include <PvDeviceU3V.h>
#include <PvStreamU3V.h>
#include <PvPipeline.h>
#define BUFFER_COUNT ( 40 )					 // ����Ԥ����Ļ���ռ��С��
#define MAX_ACQUIRE_NUM (BUFFER_COUNT*3)     // ��Ԥ����Ļ���ռ��С�£�һ���ܴ洢��ͼƬ����

using mat = cv::Mat;
using vector_m = std::vector<mat>;


class JAI
{
private:
	int heigth_;
	int width_;
	string name_;
	
	bool isConnected_ = 0;			  // �豸�����ӱ�ʶ
	bool isexistStreamPipeline_ = 0;  // �豸stream pipeline���ڱ�ʶ
	int isTriggerModeOn_ = -1;		  // ����ģʽѡ��
	bool isStart_ = 0;		          // �ɼ�״̬��ʶ
	
	int pre_acquire_img_num_ = 1;	  // ��Ҫ�ɼ���ͼ��������Ԥ��Ϊ1
	int acquire_img_all_ = 0;	      // �ɼ�ͼ������
	
	PvResult result_;
	PvString connectID_;
	PvDevice* device_ = NULL;
	PvStream* stream_ = NULL;
	PvPipeline* pipeline_ = NULL;
	vector_m imgs_;

public:
	JAI() {}
	JAI(const string& name)
		:name_(name) {}
	JAI(const string& name, int width = 0, int heigth = 0)
		:name_(name), width_(width), heigth_(heigth) {}

	bool ConnectDevice();		// �������
	void DisconnectDevice();	// �Ͽ����
	bool ConfigureDevice(int trigger_mode);		// �����������ģʽ
	bool StartAcquireImages();	// ��ʼ�ɼ�
	void StopAcquireImages();	// ֹͣ�ɼ�

	void SetPreAcquireImgNum(int pre_acquire_img_num);	// ���������ģʽΪ�ⴥ��ʱ����Ҫ��������Ԥ�����ͼ������

	vector_m GetImgs();			// ��ȡ�洢��ͼ��

protected:
	bool FindAndSelectDevice();
	bool CreateStreamAndPipeline();
	void ConfigureTriggerOn();
	void ConfigureTriggerOff();
	void GetImgsFromBuffer();	// �ӻ������л�ȡͼ�񣬲����������
	void ClearStreamAndPipeline();
	void ClearImgs();			// ����洢��ͼƬ
	void ClearDevice();
};