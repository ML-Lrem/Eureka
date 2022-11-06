#pragma once
#include <windows.h>				// 用于jai 判断平台（程序移植）
#include <iostream>
#include <opencv.hpp>
#include <vector>

#include <PvSampleUtils.h>
#include <PvSystem.h>
#include <PvDeviceU3V.h>
#include <PvStreamU3V.h>
#include <PvPipeline.h>
#define BUFFER_COUNT ( 40 )					 // 设置预分配的缓冲空间大小，
#define MAX_ACQUIRE_NUM (BUFFER_COUNT*3)     // 在预分配的缓冲空间大小下，一共能存储的图片张数

using mat = cv::Mat;
using vector_m = std::vector<mat>;


class JAI
{
private:
	int heigth_;
	int width_;
	string name_;
	
	bool isConnected_ = 0;			  // 设备已连接标识
	bool isexistStreamPipeline_ = 0;  // 设备stream pipeline存在标识
	int isTriggerModeOn_ = -1;		  // 触发模式选择
	bool isStart_ = 0;		          // 采集状态标识
	
	int pre_acquire_img_num_ = 1;	  // 想要采集的图像数量，预设为1
	int acquire_img_all_ = 0;	      // 采集图像总数
	
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

	bool ConnectDevice();		// 连接相机
	void DisconnectDevice();	// 断开相机
	bool ConfigureDevice(int trigger_mode);		// 配置相机工作模式
	bool StartAcquireImages();	// 开始采集
	void StopAcquireImages();	// 停止采集

	void SetPreAcquireImgNum(int pre_acquire_img_num);	// 当配置相机模式为外触发时，需要事先设置预拍摄的图像数量

	vector_m GetImgs();			// 获取存储的图像

protected:
	bool FindAndSelectDevice();
	bool CreateStreamAndPipeline();
	void ConfigureTriggerOn();
	void ConfigureTriggerOff();
	void GetImgsFromBuffer();	// 从缓冲区中获取图像，并清除缓冲区
	void ClearStreamAndPipeline();
	void ClearImgs();			// 清除存储的图片
	void ClearDevice();
};