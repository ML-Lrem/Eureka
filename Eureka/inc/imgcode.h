#pragma once
#include <iostream>
#include <opencv.hpp>
#include <vector>
#include "global_name.h"

class imgcode
{
private:
	int width_;
	int height_;
	int num_gray_;
	int num_phase_;
	int period_;
public:
	imgcode(int width, int height, int num_gary, int num_phase)										// height(row), width(col)
		:width_(width), height_(height), num_gray_(num_gary), num_phase_(num_phase) {}

	vector_m GreatMaskCode();											// ���ɺڰ�����(һ��һ��)
	vector_m GreatGrayCode();		// ����num_bitλ������
	vector_m GreatPhaseCode();		// ����num_step��������,�����������λ���й�'
	mat GreatChecker(int distance);

	int GetPeriod();

	void SetNumGray(int num_gray);
	void SetNumPhase(int num_phase);
};