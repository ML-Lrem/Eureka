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

	vector_m GreatMaskCode();											// 生成黑白遮罩(一黑一白)
	vector_m GreatGrayCode();		// 生成num_bit位格雷码
	vector_m GreatPhaseCode();		// 生成num_step步相移码,周期与格雷码位数有关'
	mat GreatChecker(int distance);

	int GetPeriod();

	void SetNumGray(int num_gray);
	void SetNumPhase(int num_phase);
};