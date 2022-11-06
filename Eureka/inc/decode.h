#pragma once
#include <iostream>
#include <opencv.hpp>
#include <vector>
#include "global_name.h"
#include "hardware.h"
vector_m DecodeMaskAndTherod(vector_m& bw_photos);				// 解遮罩
mat GrayPhaseDecode(vector_m& gray_photos, vector_m& phase_photos, vector_m& mask_therod, int period);			// 格雷+相移
mat MultiFrequencyHeterodyne(vector_m& phase_photos, mat& mask, int period, int f1, int f2, int f3, int n);								// 多频外差
vector_m photoundistort(scanner& scanner_user,vector_m& photo); //去畸变
