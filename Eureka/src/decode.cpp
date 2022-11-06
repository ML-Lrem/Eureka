#include "decode.h"
#include "img_preprocess.hpp"
#include "global_name.h"
#include "hardware.h"

vector_m DecodeGray(vector_m& gray_photos, vector_m& mask_therod);
mat DecodeWrappedPhase(vector_m& phase_photos, mat& mask, int period);
mat DecodeUnWrappedPhase(vector_m& gray_decoded, mat& wrapped_phase_decoded, int period);

/* �ڰ� -> ��ֵ */
vector_m DecodeMaskAndTherod(vector_m& bw_photos)
{
	mat mask;
	mat black;
	mat white;

	black = bw_photos[0]; white = bw_photos[1];
	mat therod = (black + white) / 2;

	mat sub = white - black;											// �ڰ������ȥ����Χ���ӵ�
	mask = sub;
	//ipo::ThresholdAuto(sub, mask, ipo::MASKVAL);						// ����Ӧ��ֵ��ֵ��(0,1)���ڰ׾�ֵ��ֵ,����Ϊ"̰��-����ٽ���Ҳ�������"��
	mask.forEach<uchar>([&therod](uchar& val, const int* pos) {
		if (val > therod.ptr(pos[0])[pos[1]]) { val = 1; }
		else { val = 0; }
		});

	vector_m mask_therod;
	mask_therod.emplace_back(mask);
	mask_therod.emplace_back(therod);

#ifdef DECODEIMGW
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//mask.bmp", mask * 255);
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//therod.bmp", therod);
#endif
	return mask_therod;
}

/* ������������ */
mat GrayPhaseDecode(vector_m& gray_photos, vector_m& phase_photos, vector_m& mask_therod, int period)
{
	LOG_STRING("\n���ڽ��롭��\n");
	vector_m gray_decoded = DecodeGray(gray_photos, mask_therod);
	mat wrapped_phase_decoded = DecodeWrappedPhase(phase_photos, mask_therod[0], period);
	mat unwrapped_phase_decoded = DecodeUnWrappedPhase(gray_decoded, wrapped_phase_decoded, period);

	return unwrapped_phase_decoded;
}

/* ��Ƶ��� */
mat MultiFrequencyHeterodyne(vector_m& phase_photos, mat& mask, int period, int f1, int f2, int f3, int n)
{
	//f1 f2 f3 Ϊ����Ƶ��
	int height = phase_photos[0].rows;
	int width = phase_photos[0].cols;

	int m = phase_photos.size();

	if (m % n != 0) 
	{
		std::cout << "�������" << std::endl;
		mat res;
		return res;
	}

	int num = m / n;

	std::vector<mat> wrappingPhase;
	for (int i = 0; i < num; i++) {
		std::vector<mat> temp;
		for (int j = 0; j < n; j++) {
			temp.push_back(phase_photos[i * n + j]);
		}
		wrappingPhase.push_back(DecodeWrappedPhase(temp, mask, period));
		std::vector<mat>().swap(temp);
	}

	mat PH1 = wrappingPhase[0].clone();
	mat PH2 = wrappingPhase[1].clone();
	mat PH3 = wrappingPhase[2].clone();

	cv::imwrite("D://Eureka//Eureka//output//measure//decode//PH1.tif ", PH1);
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//PH2.tif ", PH2);
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//PH3.tif ", PH3);

	mat PH12 = mat::zeros(PH1.size(), CV_32FC1);
	PH12.forEach<float>([&PH1, &PH2](float& val, const int* pos) {
		const float temp = PH2.ptr<float>(pos[0])[pos[1]] - PH1.ptr<float>(pos[0])[pos[1]];
		if (temp >= 0) val = temp;
		else val = 2 * PI + temp;
	});

	cv::imwrite("D://Eureka//Eureka//output//measure//decode//PH12.tif ", PH12);

	mat PH23 = mat::zeros(PH2.size(), CV_32FC1);
	PH23.forEach<float>([&PH3, &PH2](float& val, const int* pos) {
		const float temp = PH3.ptr<float>(pos[0])[pos[1]] - PH2.ptr<float>(pos[0])[pos[1]];
		if (temp >= 0) val = temp;
		else val = 2 * PI + temp;
	});

	cv::imwrite("D://Eureka//Eureka//output//measure//decode//PH23.tif ", PH23);

	mat PH123 = mat::zeros(PH1.size(), CV_32FC1);
	PH123.forEach<float>([&PH12, &PH23](float& val, const int* pos) {
		const float temp = PH23.ptr<float>(pos[0])[pos[1]] - PH12.ptr<float>(pos[0])[pos[1]];
		if (temp >= 0) val = temp;
		else val = 2 * PI + temp;
	});

	cv::imwrite("D://Eureka//Eureka//output//measure//decode//PH123.tif ", PH123);

	//Ƶ�ʲ�
	double f12 = f2 - f1;
	double f23 = f3 - f2;
	double f123 = f23 - f12;

	//�����23����PH23�ľ�����λ
	double R = f23 / f123;
	mat Nwrap = mat::zeros(PH123.size(), CV_32FC1);
	Nwrap.forEach<float>([&PH123, &PH23, &R](float& val, const int* pos) {
		val = floor(0.5 + (R * PH123.ptr<float>(pos[0])[pos[1]] - PH23.ptr<float>(pos[0])[pos[1]]) / (2 * PI));
	});

	mat PH23UnWrap = 2 * PI * Nwrap + PH23;

	cv::imwrite("D://Eureka//Eureka//output//measure//decode//PH23UnWrap.tif ", PH23UnWrap);

	R = f2 / f23;
	mat Nwrap1 = mat::zeros(PH23UnWrap.size(), CV_32FC1);
	Nwrap1.forEach<float>([&PH23UnWrap, &PH2, &R](float& val, const int* pos) {
		val = floor(0.5 + (R * PH23UnWrap.ptr<float>(pos[0])[pos[1]] - PH2.ptr<float>(pos[0])[pos[1]]) / (2 * PI));
	});
	mat PH2UnWrap = 2 * PI * Nwrap1 + PH2;

	PH2UnWrap = PH2UnWrap * period / (2 * PI);
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//UnWrap.tif ", PH2UnWrap);

	return PH2UnWrap;
}



/************************* �ڲ�������ʹ��Protect�� ***********************/
/* ������� */
vector_m DecodeGray(vector_m& gray_photos, vector_m& mask_therod)
{
	int height = gray_photos[0].rows;                                                       // �������ͼƬ��ʵ�ʳߴ�
	int width = gray_photos[0].cols;
	
	int test_channel = gray_photos[0].channels();											// ȷ��һ���ǵ�ͨ��

	mat empty_temp = mat::zeros(height, width, CV_8UC1);										// �����հ�ģ��
	mat gray_decoded_k12 = mat::zeros(height, width, CV_8UC1);

	mat mask = mask_therod[0];
	mat therod = mask_therod[1];
	//std::cout << gray_photos.size() << std::endl;
	/* ����ǰN�Ž��K12 */
	for (int i = 0; i < gray_photos.size(); i++)
	{
		//ipo::ThresholdAuto(gray_photos[i], gray_photos[i],ipo::MASKVAL);						// ����Ӧ��ֵ��ֵ����Ĭ��MASKģʽ�����Ż���
		//ToDo

		gray_photos[i].forEach<uchar>([&therod](uchar& val, const int* pos) {
			if (val > therod.ptr(pos[0])[pos[1]]) { val = 1; }
			else { val = 0; }
			});

		//string dst = "D://Eureka//Eureka//output//measure//decode//gray_photos" + std::to_string(i) + ".bmp";;
		//cv::imwrite(dst, gray_photos[i] * 255);

		cv::bitwise_xor(empty_temp, gray_photos[i], empty_temp);								// �������
		gray_decoded_k12 += empty_temp * pow(2, gray_photos.size() - 1 - i);
	}

	/* ����Mask���� */
	cv::multiply(gray_decoded_k12, mask, gray_decoded_k12);

	/* ���ݵ�N�Ż����������K1���K2 */
	mat gray_decoded_k1 = mat::zeros(height, width, CV_8UC1);
	mat gray_decoded_k2 = mat::zeros(height, width, CV_8UC1);

	gray_decoded_k1.forEach<uchar>([&gray_decoded_k12](uchar& val, const int* pos) {
		val = (uchar)gray_decoded_k12.ptr<uchar>(pos[0])[pos[1]] / 2;
		});

	gray_decoded_k2.forEach<uchar>([&gray_decoded_k12](uchar& val, const int* pos) {
		val = (uchar)((gray_decoded_k12.ptr<uchar>(pos[0])[pos[1]] + 1) / 2);
		});

	vector_m gray_decoded;
	gray_decoded.emplace_back(gray_decoded_k1);
	gray_decoded.emplace_back(gray_decoded_k2);

#ifdef DECODEIMGW
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//gray_decoded_k12.bmp", gray_decoded_k12 );
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//gray_decoded_k1.bmp", gray_decoded_k1 );
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//gray_decoded_k2.bmp", gray_decoded_k2 );
#endif

	return gray_decoded;
}

/* �������λ��4�����ƣ� */
mat DecodeWrappedPhase(vector_m& phase_photos, mat& mask, int period)
{
	int height = phase_photos[0].rows;
	int width = phase_photos[0].cols;

	mat wrapped_phase_decoded = mat::zeros(height, width, CV_32FC1);									// 8λ��������1ͨ��

	wrapped_phase_decoded.forEach <float>([&phase_photos, &period, &mask](float& val, const int* pos) {			// ������
		if (mask.ptr<uchar>(pos[0])[pos[1]]) {														// pos[0]->rows; pos[1]->cols
			const uchar i0_pixel = phase_photos[0].ptr<uchar>(pos[0])[pos[1]];
			const uchar i1_pixel = phase_photos[1].ptr<uchar>(pos[0])[pos[1]];
			const uchar i2_pixel = phase_photos[2].ptr<uchar>(pos[0])[pos[1]];
			const uchar i3_pixel = phase_photos[3].ptr<uchar>(pos[0])[pos[1]];

			val = atan2(i3_pixel - i1_pixel, i0_pixel - i2_pixel);							// val \in (-PI,PI] ������ʽ�����Ż�������������

			if (val < 0) { val = val + 2 * PI; }											// ��(-PI,PI]�任��(0,2PI]

			// gamma����
		}
		});

#ifdef DECODEIMGW
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//wrapped_phase_decoded.bmp", wrapped_phase_decoded);
	mat wrapped_phase_decoded_float;
	wrapped_phase_decoded.convertTo(wrapped_phase_decoded_float, CV_32FC1);
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//wrapped_phase_decoded.tif", wrapped_phase_decoded_float);
#endif

	return wrapped_phase_decoded;
}

/*  �������λ */
mat DecodeUnWrappedPhase(vector_m& gray_decoded, mat& wrapped_phase_decoded, int period)
{
	/* �⻥�������� */
	/* �������ݽṹӦ�ÿ����Ż���val�������û��Ǹ������� */
	int height = gray_decoded[0].rows;
	int width = gray_decoded[0].cols;
	//std::cout << period << std::endl;


	mat unwrapped_phase_decoded = mat::zeros(height, width, CV_32FC1);					// 16λ��������ζ�Ź�դ��ȵķ�ΧΪ:[0,65535]

	mat gray_decoded_k1 = gray_decoded[0];
	mat gray_decoded_k2 = gray_decoded[1];

	unwrapped_phase_decoded.forEach<float>([&wrapped_phase_decoded, &gray_decoded_k1, gray_decoded_k2, &period](float& val, const int* pos) {
		if (wrapped_phase_decoded.ptr<float>(pos[0])[pos[1]] <= PI / 2)		// 0 - pi/2
		{
			val = wrapped_phase_decoded.ptr<float>(pos[0])[pos[1]] + gray_decoded_k2.ptr<uchar>(pos[0])[pos[1]] * 2 * PI;
		}
		else if (wrapped_phase_decoded.ptr<float>(pos[0])[pos[1]] >= 3 * PI / 2)	// 3pi/2 - 2pi
		{
			val = wrapped_phase_decoded.ptr<float>(pos[0])[pos[1]] + gray_decoded_k2.ptr<uchar>(pos[0])[pos[1]] * 2 * PI - 2 * PI;
		}
		else			// pi/2 - 3pi/2
		{
			val = wrapped_phase_decoded.ptr<float>(pos[0])[pos[1]] + gray_decoded_k1.ptr<uchar>(pos[0])[pos[1]] * 2 * PI;
		}
		val = val * period / (2 * PI);

		});

#ifdef DECODEIMGW
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//unwrapped_phase_decoded.bmp", unwrapped_phase_decoded);
	mat unwrapped_phase_decoded_float;
	unwrapped_phase_decoded.convertTo(unwrapped_phase_decoded_float, CV_32FC1);
	cv::imwrite("D://Eureka//Eureka//output//measure//decode//unwrapped_phase_decoded_test.tif", unwrapped_phase_decoded_float);
#endif

	return unwrapped_phase_decoded;
}


//ȥ����
vector_m photoundistort(scanner& scanner_user,vector_m& photo)
{
	//��ȡ�������
	mat cam_k = scanner_user.scanner_camera.GetInstrisinc();
	//cout << cam_k << endl;
	mat cam_d = scanner_user.scanner_camera.GetDistortion();
	//cout << cam_d << endl;

	vector_m ud_photo;

	for (int i = 0; i < photo.size(); i++)
	{
		mat ud_p;
		cv::undistort(photo[i], ud_p, cam_k, cam_d);
		ud_photo.emplace_back(ud_p);
		string dst;
		//if (photo.size() == 2)
		//{
		//	dst = "D://Eureka//Eureka//output//measure//camera//undistort//mask//" + std::to_string(i) + ".bmp";
		//	cv::imwrite(dst, ud_p);
		//}
		//else if (photo.size() == 8)
		//{
		//	dst = "D://Eureka//Eureka//output//measure//camera//undistort//gray//" + std::to_string(i) + ".bmp";
		//	cv::imwrite(dst, ud_p);
		//}
		//else if (photo.size() == 4)
		//{
		//	dst = "D://Eureka//Eureka//output//measure//camera//undistort//phase//" + std::to_string(i) + ".bmp";
		//	cv::imwrite(dst, ud_p);
		//}
	}

	return ud_photo;
}