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

// ����࣬�̳�����豸JAI
class camera: public JAI
{
private:
	const string name_;						// �������
	int width_;								// ����-��
	int height_;							// ����-��
	mat instrisinc_;						// �ڲ� double
	mat extrinsics_;						// ���
	mat distortion_;						// ����ϵ��
	mat calibrate_error_;					// �궨���: ��ͶӰ���������
public:
	camera(const string& name, int width = 0, int height = 0)
		:name_(name), width_(width), height_(height) {}

	vector_m Calibrate(vector_m& photos, CalibrateConfig& calcon);	// �궨

	void SetALLCalibrateParam(vector_m params);

	const string GetName() const;			// ��ȡ��˽�в���
	mat GetInstrisinc() const;
	mat GetExtrinsics() const;
	mat GetDistortion() const;
	int GetWidth() const;
	int GetHeight() const;
};


// ͶӰ����
class projector: public DLP
{
private:
	const string name_;						// ����
	int width_;								// ����-��(��֪)
	int height_;							// ����-��(��֪)
	mat instrisinc_;						// �ڲ�
	mat extrinsics_;						// ���
	mat distortion_;						// ����ϵ��
	mat calibrate_error_;					// �궨���: ��ͶӰ���������...
public:
	projector(const string& name, int width = 0, int height = 0)
		:name_(name), width_(width), height_(height) {}

	void Calibrate(vector_m& photos, vector_m& W2C_rt, camera& camera_calibrated, CalibrateConfig& calcon); 		// �궨

	void SetALLCalibrateParam(vector_m params);

	const string GetName() const;			// ��ȡ��˽�в���
	mat GetInstrisinc() const;
	mat GetExtrinsics() const;
	mat GetDistortion() const;
	int GetWidth() const;
	int GetHeight() const;
protected:
	Point3f SolveCam3dPoint(Point2f feature_2d_point, mat instrisinc, mat extrinsics);
	mat SolveC2Prt(vector_m W2C_rt, vector_m W2P_rt);
};



// ɨ������
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





