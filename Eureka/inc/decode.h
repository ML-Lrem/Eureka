#pragma once
#include <iostream>
#include <opencv.hpp>
#include <vector>
#include "global_name.h"
#include "hardware.h"
vector_m DecodeMaskAndTherod(vector_m& bw_photos);				// ������
mat GrayPhaseDecode(vector_m& gray_photos, vector_m& phase_photos, vector_m& mask_therod, int period);			// ����+����
mat MultiFrequencyHeterodyne(vector_m& phase_photos, mat& mask, int period, int f1, int f2, int f3, int n);								// ��Ƶ���
vector_m photoundistort(scanner& scanner_user,vector_m& photo); //ȥ����
