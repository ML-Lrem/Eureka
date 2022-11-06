#pragma once
#include <Windows.h>
#include <iostream>
#include <opencv.hpp>
#include <imgproc.hpp>
#include <vector>
#include "global_name.h"
#include "file.h"

#include "jai.h"
#include "dlp.h"

// 相机类，继承相机设备JAI
class camera: public JAI
{
private:
	const string name_;						// 相机名称
	int width_;								// 像素-宽
	int height_;							// 像素-高
	mat instrisinc_;						// 内参 double
	mat extrinsics_;						// 外参
	mat distortion_;						// 畸变系数
	mat calibrate_error_;					// 标定误差: 重投影误差、距离误差
public:
	camera(const string& name, int width = 0, int height = 0)
		:name_(name), width_(width), height_(height) {}

	vector_m Calibrate(vector_m& photos, CalibrateConfig& calcon);	// 标定

	void SetALLCalibrateParam(vector_m params);

	const string GetName() const;			// 获取类私有参数
	mat GetInstrisinc() const;
	mat GetExtrinsics() const;
	mat GetDistortion() const;
	int GetWidth() const;
	int GetHeight() const;
};


// 投影仪类
class projector: public DLP
{
private:
	const string name_;						// 名称
	int width_;								// 像素-宽(已知)
	int height_;							// 像素-高(已知)
	mat instrisinc_;						// 内参
	mat extrinsics_;						// 外参
	mat distortion_;						// 畸变系数
	mat calibrate_error_;					// 标定误差: 重投影误差、距离误差...
public:
	projector(const string& name, int width = 0, int height = 0)
		:name_(name), width_(width), height_(height) {}

	void Calibrate(vector_m& photos, vector_m& W2C_rt, camera& camera_calibrated, CalibrateConfig& calcon); 		// 标定

	void SetALLCalibrateParam(vector_m params);

	const string GetName() const;			// 获取类私有参数
	mat GetInstrisinc() const;
	mat GetExtrinsics() const;
	mat GetDistortion() const;
	int GetWidth() const;
	int GetHeight() const;
protected:
	Point3f SolveCam3dPoint(Point2f feature_2d_point, mat instrisinc, mat extrinsics);
	mat SolveC2Prt(vector_m W2C_rt, vector_m W2P_rt);
};



// 扫描仪类
class scanner
{
private:
	const string name_;
	int scan_times_ = 0;
	double running_time_;
public:
	camera scanner_camera;
	projector scanner_projector;
	scanner(const string& name, camera& camera_user, projector& projector_user)
		:name_(name), scanner_camera(camera_user), scanner_projector(projector_user) {}

	bool Connect();
	bool Configure();
	void SetWorkSpace();
	void Calibrate(CalibrateConfig& calcon);
	vector_m StartScan();
	void StopScan();
	void Disconnect();

	const double GetRunningTime() const;
	const string GetName() const;
};





