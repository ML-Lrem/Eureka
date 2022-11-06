/* ����ʹ��ע���������README.md */

#include <iostream>
#include <opencv.hpp>
#include <core/utils/logger.hpp>
#include <time.h>
#include <vector>
#include "global_name.h"

#include "imgcode.h"
#include "hardware.h"
#include "decode.h"
#include "triangulation.h"
#include "file.h"

vector_m CutVectorMat(vector_m& img_vetor, int begin, int num);

// ɨ����򡪡�Ӳ������
int main()
{
	/* ֻ���������־ */
	cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

	/* �½������ͶӰ�Ǻ�ɨ������ */
	camera eureka_camera("Eureka_v1_camera", 2560, 2048);
	projector eureka_projector("Eureka_v1_projector", 912, 1140);
	scanner eureka("Eureka_v1", eureka_camera, eureka_projector);

	/* ����ɨ���� */
	if (eureka.Connect())
	{
		std::cout << eureka.GetName() << ":is connected -> " << std::endl;
	}
	else { return 0; }

	/* ��ʼ��ɨ���� */
	if (eureka.Configure())
	{
		std::cout << eureka.GetName() << ": is configured -> " << std::endl;
	}
	else { return 0; }

	/* ���������ռ� */
	bool isChangeWorkspace;
	LOG_STRING("Change work space��Yes(1) or Not(0)");
	std::cin >> isChangeWorkspace;
	if (isChangeWorkspace)
	{
		eureka.SetWorkSpace();
	}

	/* ����궨 */
	CalibrateConfig calcon;	
	eureka.Calibrate(calcon);

	/* ��ʼɨ�� */
	int scan_status = SCAN_STOP; // ��¼ɨ��״̬
	while (1)
	{
		LOG_STRING("Start scanning. Yes(1) | Not(0) | Pause(2)");
		std::cin >> scan_status;

		if(scan_status == SCAN_START)
		{
			vector_m scan_photos = eureka.StartScan();					  // ɨ��
			std::cout << "Scanning running time:" << eureka.GetRunningTime() << "ms" << std::endl;   // ��ȡɨ������ʱ��

			vector_m mask_photos = CutVectorMat(scan_photos, INDEX_MASK, NUM_MASK);
			vector_m gray_photos = CutVectorMat(scan_photos, INDEX_GRAY, NUM_GRAY);
			vector_m phase_photos = CutVectorMat(scan_photos, INDEX_PHASE, NUM_PHASE);

			//ȥ�������
			vector_m ud_mask_photos = photoundistort(eureka, mask_photos);
			vector_m ud_gray_photos = photoundistort(eureka, gray_photos);
			vector_m ud_phase_photos = photoundistort(eureka, phase_photos);
			vector_m mask_therod = DecodeMaskAndTherod(ud_mask_photos);
			mat photos_decoded = GrayPhaseDecode(ud_gray_photos, ud_phase_photos, mask_therod, PERIOD);
			
			//ȥͶӰ�ǻ��䲢�ؽ�
			mat point_camera_XYZ = udTriangulation(eureka, photos_decoded, mask_therod[0]);
			LOG_STRING("��mat���ݴ洢Ϊtxt����");
			f::WriteMatToFile(point_camera_XYZ, "point_camera_XYZ");  // �洢��������
		}
		else if(scan_status == SCAN_STOP)
		{
			eureka.scanner_camera.StartAcquireImages();
			eureka.Disconnect();
			LOG_STRING("STOP, Thanks for using !");
			return 0;
		}
		else if(scan_status == SCAN_PAUSE)
		{
			eureka.StopScan();
			LOG_STRING("PAUSE !");
		}
		else
		{
			continue;
		}
	}
}

/* ���ɸ����դͼ */
//int main()
//{
//	imgcode vaster(912, 1140, NUM_GRAY, NUM_PHASE);
//	vector_m mask_code = vaster.GreatMaskCode();
//	vector_m gray_code = vaster.GreatGrayCode();
//	vector_m phase_code = vaster.GreatPhaseCode();		// ���Ǽ����ֵ�ضϼ�⣬�ж��Ƿ����������
//	int period = vaster.GetPeriod();
//
//	LOG_STRING_VAR("period", period);
//
//	f::WriteMats(mask_code, "D://Eureka//Eureka//output//measure//camera_test//mask");
//	f::WriteMats(gray_code, "D://Eureka//Eureka//output//measure//camera_test//gray");
//	f::WriteMats(phase_code, "D://Eureka//Eureka//output//measure//camera_test//phase");
//}


/* Vector(mat) ���� */
vector_m CutVectorMat(vector_m& img_vetor, int begin, int num)
{
	vector_m::const_iterator begin_img = img_vetor.begin() + begin;
	vector_m::const_iterator end_img = img_vetor.begin() + begin + num;
	vector_m cut_result(begin_img, end_img);
	return cut_result;
}