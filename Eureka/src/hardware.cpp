#include "hardware.h"
#include<math.h>




/*  ���  */
vector_m camera::Calibrate(vector_m& photos, CalibrateConfig& calcon)
{
	string points_dst;
	/* ��������洢λ�� */
	string param_filename[4];							// �������
	param_filename[0] = calcon.camera_calibrate_param_folder + "//cam_instrisinc.tif";
	param_filename[1] = calcon.camera_calibrate_param_folder + "//cam_extrinsics.tif";
	param_filename[2] = calcon.camera_calibrate_param_folder + "//cam_distortion.tif";
	param_filename[3] = calcon.camera_calibrate_param_folder + "//cam_calibrate_error.tif";

	/* ��ȡͼ��ߴ� */
	cv::Size cam_size(width_, height_);

	/* 2d(O_uv): ��ȡ�궨ͼ�и������㼰���ά��Ϣ(��ȡ�ǵ�) (ͼ����������ϵ) */
	LOG_STRING("������ȡ�����㡭��\n");
	v_vector_2f total_feature_2d_points;												// �궨���ϵĶ�ά�ǵ����꡾����Ԥ�����ַ��
	cv::Size points_num(calcon.num_board_point[0], calcon.num_board_point[1]);			//���ܷ��Զ���ȡ���нǵ������
	for (int i = 0; i < photos.size(); i++)
	{
		vector_2f feature_2d_points;
		cv::SimpleBlobDetector::Params params;
		params.blobColor =0;
		params.maxArea = 5000;
		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
		//���Բ�ģ�˳��Ϊ���ϵ�����
		//if (!cv::findCirclesGrid(photos[i], points_num, feature_2d_points, cv::CALIB_CB_SYMMETRIC_GRID| cv::CALIB_CB_CLUSTERING,detector))
		if (!cv::findCirclesGrid(photos[i], points_num, feature_2d_points, cv::CALIB_CB_SYMMETRIC_GRID))
		//���ǵ㣬˳������ϵ�����
		//if (!cv::findChessboardCornersSB(photos[i], points_num, feature_2d_points, cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY))			
		{
			LOG_STRING_VAR("��������ȡʧ��", i);
		}
		else
		{
			if (i == 0) { std::cout << "��������ȡ�ɹ�:\t"; }
			std::cout << i << "\t";
			if (i == photos.size() - 1) { std::cout << std::endl; }

			total_feature_2d_points.emplace_back(feature_2d_points);

			points_dst = "D://Eureka//Eureka//output//calibrate//points//feature_cam" + std::to_string(i) + ".txt";
			f::Write2dPointsToFile(feature_2d_points, points_dst);

			//�ڱ궨ͼƬ�ϱ����ȡ���Ľǵ�
			cv::drawChessboardCorners(photos[i], points_num, feature_2d_points, true); //������ͼƬ�б�ǽǵ�
			string dst;
			dst= "D://Eureka//Eureka//output//calibrate//camera_photos//�ǵ���ȡ//" +std::to_string(i) + ".bmp";
			cv::imwrite(dst, photos[i]);
		}
	}

	/* 3d(O_w): �������ж�ά�ǵ��Ӧ����ά��Ϣ��Zw=0 (��������ϵ) */
	LOG_STRING("�����Ӧ����ά���ꡭ��");
	v_vector_3f total_feature_3d_points;						// �궨���ϵ���ά�ǵ�����
	vector_3f  feature_3d_points;
	for (int i = 0; i < total_feature_2d_points.size(); i++)
	{
		if (i == 0)
		{
			Point3f feature_3d_point;

			//�������
			for (int y = 1; y <= points_num.height; y++)			// ÿ��ͼ�нǵ�����
			{
				for (int x = 1; x <= points_num.width; x++)
				{
					feature_3d_point.x = x * calcon.distance_board_point[0];				// ��ǰ��ά�ǵ��Ӧ����ά������
					feature_3d_point.y = y * calcon.distance_board_point[1];
					feature_3d_point.z = 0;
					feature_3d_points.emplace_back(feature_3d_point);
				}
			}
			points_dst = "D://Eureka//Eureka//output//calibrate//points//board_cam" + std::to_string(i) + ".txt";
			f::Write3dPointsToFile(feature_3d_points, points_dst);
		}
		total_feature_3d_points.emplace_back(feature_3d_points);
	}

	/* opencv�궨 */
	extrinsics_ = mat::zeros(3, 4, CV_64FC1);		// �������ϵ��Ϊ��������ϵ�������Ϊ0����
	vector_m rvecs;
	vector_m tvecs;
	mat extrinsics_m;
	
	double calibrate_error =cv::calibrateCamera(total_feature_3d_points, total_feature_2d_points, cam_size, instrisinc_, distortion_, rvecs, tvecs, cv::CALIB_FIX_K3| cv::CALIB_ZERO_TANGENT_DIST);//CALIB_ZERO_TANGENT_DIST��ѡ
	cout << calibrate_error << endl;
	/*������ͶӰ���*/
	//��ʼ���ǵ�����
	LOG_STRING("�궨��ɣ���ʼ���۱궨���");
	int i = 0;
	int num = int(total_feature_3d_points.size());
	vector<int> point_counts;  // ÿ��ͼ���нǵ������
	for (i = 0; i < num; i++)
	{
		point_counts.push_back(points_num.width * points_num.height);
	}

	vector<Point2f> total_feature_2d_points2;//�������¼���õ���ͶӰ��
	vector<double> cam_errors;	// �궨��� (��ͶӰ������������)	
	double cam_error_ = 0;
	int totalPoints = 0;
	double totalErr = 0;

	for (i = 0; i < num; i++)
	{
		vector<Point3f> tempPointSet = total_feature_3d_points[i];
		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
		projectPoints(tempPointSet, rvecs[i], tvecs[i],
			instrisinc_, distortion_, total_feature_2d_points2);
		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
		vector<Point2f> tempImagePoint = total_feature_2d_points[i];
		mat tempImagePointMat = mat(1, tempImagePoint.size(), CV_32FC2);
		mat image_points2Mat = mat(1, total_feature_2d_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(total_feature_2d_points2[j].x, total_feature_2d_points2[j].y);
			tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		cam_error_ = norm(total_feature_2d_points[i], total_feature_2d_points2, cv::NORM_L2);
		totalErr += cam_error_ /= point_counts[i];
		cam_errors.emplace_back(cam_error_);
		std::cout << "��" << i + 1 << "��ͼ���ƽ����" << cam_error_ << "����" << endl;
	}

	std::cout << "����ƽ����" << totalErr / num << "����" << endl;
	points_dst = "D://Eureka//Eureka//output//calibrate//cam_error.txt";
	f::WriteErrorToFile(totalErr / num, points_dst);

	/* �궨����ת�� */
	vector_m W2C_rt;										// ������ʵ��Ԥ�����ڴ桿
	string cam_board_rt_dst;
	for (int i = 0; i < total_feature_2d_points.size(); i++)
	{
		mat extrinsics_temp = mat::zeros(3, 5, CV_64FC1);

		cv::Rodrigues(rvecs[i], rvecs[i]);		// ��ת����ת������ת����
		extrinsics_temp.colRange(0, 3) = extrinsics_temp.colRange(0, 3) + rvecs[i];
		extrinsics_temp.colRange(3, 4) = extrinsics_temp.colRange(3, 4) + tvecs[i];
		extrinsics_temp.at<double>(0,4)= cam_errors[i];
		extrinsics_temp.at<double>(1, 4) = 1/cam_errors[i];
		cam_board_rt_dst = "D://Eureka//Eureka//output//calibrate//cam_board//cam_board_rt"+std::to_string(i) + ".txt";
		//cout << extrinsics_temp << endl;
		f::WriteMat(extrinsics_temp, cam_board_rt_dst);
		W2C_rt.emplace_back(extrinsics_temp);
	}
	/* ���۱궨���� */
	LOG_STRING_VAR("camera_calibrated.GetInstrisinc()", instrisinc_);
	LOG_STRING_VAR("camera_calibrated.GetDistortion()", distortion_);
	LOG_STRING("");

	/* �洢�궨��� */
	cv::imwrite(param_filename[0], instrisinc_);
	cv::imwrite(param_filename[1], extrinsics_);
	cv::imwrite(param_filename[2], distortion_);
	cv::imwrite(param_filename[3], totalErr / num);

	return W2C_rt;
}

void camera::SetALLCalibrateParam(vector_m params)
{
	instrisinc_ = params[3];			// �ֵ���instrisinc < extrinsics < distortion < calibrate_error -> 3,2,1,0
	extrinsics_ = params[2];
	distortion_ = params[1];
	calibrate_error_ = params[0];
}

const string camera::GetName() const { return name_; }
mat camera::GetInstrisinc() const { return instrisinc_; }
mat camera::GetExtrinsics() const { return extrinsics_; }
mat camera::GetDistortion() const { return distortion_; }
int camera::GetWidth() const { return width_; }
int camera::GetHeight() const { return height_; }



/*  ͶӰ��
*
*/
void projector::Calibrate(vector_m& photos, vector_m& W2C_rt, camera& camera_calibrated, CalibrateConfig& calcon)
{
	string points_dst;
	/* ͶӰ�ǲ����洢λ�� */
	string param_filename[4];							// ͶӰ�ǲ���
	param_filename[0] = calcon.projector_calibrate_param_folder + "//pro_instrisinc.tif";
	param_filename[1] = calcon.projector_calibrate_param_folder + "//pro_extrinsics.tif";
	param_filename[2] = calcon.projector_calibrate_param_folder + "//pro_distortion.tif";
	param_filename[3] = calcon.projector_calibrate_param_folder + "//pro_calibrate_error.tif";

	/* ��ȡ���̸�ͶӰͼ�ߴ� */
	cv::Size pro_size(width_, height_);

	//for (int i = 0; i < photos.size(); i++)
	//{
	//	cv::threshold(photos[i], photos[i], 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	//}
	

	//ʹ�����������ͶӰ�Ǳ궨ͼ��ȥ����
	vector_m ud_photos;
	for (int i = 0; i < photos.size(); i++)
	{
		mat ud_p;
		cv::undistort(photos[i], ud_p, camera_calibrated.GetInstrisinc(), camera_calibrated.GetDistortion());
		string dst;
		dst = "D://Eureka//Eureka//output//calibrate//projector_photos//ȥ�����//" + std::to_string(i) + ".bmp";
		cv::imwrite(dst, ud_p);
		ud_photos.emplace_back(ud_p);
	}


	/* Ѱ���������ͼƬ�еĶ�ά������ */
	LOG_STRING("������ȡ�����㡭��\n");
	v_vector_2f total_cam_feature_2d_points;
	vector_2f cam_feature_2d_points;
	Point2f cam_feature_2d_point;
	cv::Size points_num(calcon.num_projector_point[0], calcon.num_projector_point[1]);
	for (int i = 0; i < photos.size(); i++)
	{
		//���ǵ�
		//if (!cv::findChessboardCorners(ud_photos[i], points_num, cam_feature_2d_points, cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_FILTER_QUADS))// ��ά�����������еģ������µ�����
		if (!cv::findChessboardCornersSB(ud_photos[i], points_num, cam_feature_2d_points, cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY))			//��ά�����������У������ϵ�����
		{
			LOG_STRING_VAR("��������ȡʧ��", i);
			W2C_rt.erase(W2C_rt.begin() + i);					// �޳�(����)��Ӧ��RT����
		}
		else
		{
			if (i == 0) { std::cout << "��������ȡ�ɹ�:\t"; }
			std::cout << i << "\t";
			if (i == photos.size() - 1) { std::cout << std::endl; }
			//����ȡ�ǵ���������ؼ���ȷ��;
			//cv::cornerSubPix(ud_photos[i], cam_feature_2d_points, cv::Size(11, 11),cv::Size(-1, -1), cv::TermCriteria(0, 30, 0.1));
			//��¼�ǵ�����
			points_dst = "D://Eureka//Eureka//output//calibrate//points//feature_pro" + std::to_string(i) + ".txt";
			f::Write2dPointsToFile(cam_feature_2d_points, points_dst);
			total_cam_feature_2d_points.emplace_back(cam_feature_2d_points);// ˳�����϶��£���������
			//�ڱ궨ͼƬ�ϱ����ȡ���Ľǵ�
			cv::drawChessboardCorners(ud_photos[i], points_num, cam_feature_2d_points, true); //������ͼƬ�б�ǽǵ�
			string dst;
			dst = "D://Eureka//Eureka//output//calibrate//projector_photos//�ǵ���ȡ//" + std::to_string(i) + ".bmp";
			cv::imwrite(dst, ud_photos[i]);
		}
	}
	if (W2C_rt.size() != total_cam_feature_2d_points.size())		// ��ȫ����
	{
		LOG_STRING("��������ȡ������ʧ�ܵ�ͼ�񣬵�δ�ɹ�������Ӧ����ξ���");

		return;
	}
	else if (W2C_rt.size() < 4)
	{
		LOG_STRING("��������ȡ������ʧ�ܵ�ͼ�񣬵�δ�ɹ�������Ӧ����ξ���");
		return;
	}

	/* ʹ����������������ͼ������ϵ�µĶ�ά������ת��Ϊ��������ϵ�µ���ά������ */
	v_vector_3f total_cam_feature_3d_points;
	vector_3f cam_feature_3d_points;
	Point3f cam_feature_3d_point;
	for (int i = 0; i < total_cam_feature_2d_points.size(); i++)			// n�š���Ч�궨ͼ��
	{
		cam_feature_3d_points = {};											// ���	��Ϊʲô���ﲻ���ڡ�emplace_back���⡱��
		cam_feature_2d_points = total_cam_feature_2d_points[i];
		for (int j = 0; j < points_num.area(); j++)							// ����������
		{
			cam_feature_2d_point = cam_feature_2d_points[j];				// ��ǰ��һ����ά������
			cam_feature_3d_point = SolveCam3dPoint(cam_feature_2d_point, camera_calibrated.GetInstrisinc(), W2C_rt[i]);	    // �����ά������(�����������ϵ)��Ӧ����ά������(�������ϵ)
			cam_feature_3d_points.emplace_back(cam_feature_3d_point);

			points_dst = "D://Eureka//Eureka//output//calibrate//points//board_pro" + std::to_string(i) + ".txt";
			f::Write3dPointsToFile(cam_feature_3d_points, points_dst);
		}
		total_cam_feature_3d_points.emplace_back(cam_feature_3d_points);
	}

	/* �������̸�ͶӰ��ͼ��������(�ǵ�)�Ķ�ά���� */
	/* 3d(O_w): �������ж�ά�ǵ��Ӧ����ά��Ϣ��Zw=0 (��������ϵ) */
	LOG_STRING("�����Ӧ����ά���ꡭ��");
	v_vector_2f total_pro_feature_2d_points;									// ͶӰ�ǵĶ�ά�ǵ�����
	vector_2f  pro_feature_2d_points;
	for (int i = 0; i < total_cam_feature_2d_points.size(); i++)				// ��Ч�궨ͼ
	{
		if (i == 0)
		{
			
			Point2f feature_2d_point;											// ��ǰ��ά�ǵ��Ӧ�Ķ�ά����
			
			//�������
			//for (int y = points_num.height; y >= 1; y--)			// ÿ��ͼ�нǵ�����
			//{
			//	for (int x = points_num.width; x >= 1;  x--)
			//	{
			//		feature_2d_point.x = 96 + x * calcon.distance_projector_point[0];
			//		feature_2d_point.y = 60 + y * calcon.distance_projector_point[1];
			//		pro_feature_2d_points.emplace_back(feature_2d_point);
			//	}
			//}

			//�������
			for (int y = 1; y <= points_num.height; y++)			// ÿ��ͼ�нǵ�����
			{
				for (int x = 1; x <= points_num.width; x++)
				{
					feature_2d_point.x = 96 + x * calcon.distance_projector_point[0];
					feature_2d_point.y = 60 + y * calcon.distance_projector_point[1];
					pro_feature_2d_points.emplace_back(feature_2d_point);
				}
			}
			points_dst = "D://Eureka//Eureka//output//calibrate//points//pro" + std::to_string(i) + ".txt";
			f::Write2dPointsToFile(pro_feature_2d_points, points_dst);
		}
		total_pro_feature_2d_points.emplace_back(pro_feature_2d_points);
	}

	/* opencv �궨 */
	vector_m rvecs;
	vector_m tvecs;
	double calibrate_error_= cv::calibrateCamera(total_cam_feature_3d_points, total_pro_feature_2d_points, pro_size, instrisinc_, distortion_, rvecs, tvecs, cv::CALIB_FIX_K3| cv::CALIB_ZERO_TANGENT_DIST);//CALIB_ZERO_TANGENT_DIST��ѡ
	cout << calibrate_error_ << endl;

	/*������ͶӰ���*/
	//��ʼ���ǵ�����
	LOG_STRING("�궨��ɣ���ʼ���۱궨���");
	int i = 0;
	int num = int(total_cam_feature_3d_points.size());
	vector<int> point_counts;  // ÿ��ͼ���нǵ������
	for (i = 0; i < num; i++)
	{
		point_counts.push_back(points_num.width * points_num.height);
	}

	vector<Point2f> total_pro_feature_2d_points2;//�������¼���õ���ͶӰ��
	vector<double> prj_errors;
	double prj_error_;	// �궨��� (��ͶӰ������������)	
	int totalPoints = 0;
	double totalErr = 0;

	for (i = 0; i < num; i++)
	{
		vector<Point3f> tempPointSet = total_cam_feature_3d_points[i];
		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
		projectPoints(tempPointSet, rvecs[i], tvecs[i],
			instrisinc_, distortion_, total_pro_feature_2d_points2);
		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
		vector<Point2f> tempImagePoint = total_pro_feature_2d_points[i];
		mat tempImagePointMat = mat(1, tempImagePoint.size(), CV_32FC2);
		mat image_points2Mat = mat(1, total_pro_feature_2d_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(total_pro_feature_2d_points2[j].x, total_pro_feature_2d_points2[j].y);
			tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		prj_error_ = norm(total_pro_feature_2d_points[i], total_pro_feature_2d_points2, cv::NORM_L2);
		totalErr += prj_error_ /= point_counts[i];
		prj_errors.emplace_back(prj_error_);
		std::cout << "��" << i + 1 << "��ͼ���ƽ����" << prj_error_ << "����" << endl;
	}

	std::cout << "����ƽ����" << totalErr / num << "����" << endl;
	points_dst = "D://Eureka//Eureka//output//calibrate//pro_error.txt";
	f::WriteErrorToFile(totalErr / num, points_dst);


	/* �궨����ת�� */
	vector_m W2P_rt;	// ������ʵ��Ԥ�����ڴ桿
	string prj_board_rt_dst;
	for (int i = 0; i < total_cam_feature_2d_points.size(); i++)
	{
		mat extrinsics_temp = mat::zeros(3, 5, CV_64FC1);			// ���ǳ���Ҫ�����⣺vector���ڴ���䷽��(ʲôʱ�������������⣿)��

		cv::Rodrigues(rvecs[i], rvecs[i]);		// ��ת����ת������ת����
		extrinsics_temp.colRange(0, 3) = extrinsics_temp.colRange(0, 3) + rvecs[i];
		extrinsics_temp.colRange(3, 4) = extrinsics_temp.colRange(3, 4) + tvecs[i];
		extrinsics_temp.at<double>(0, 4) = prj_errors[i];
		extrinsics_temp.at<double>(1, 4) = 1 / prj_errors[i];
		prj_board_rt_dst = "D://Eureka//Eureka//output//calibrate//prj_board//prj_board_rt" + std::to_string(i) + ".txt";
		//cout << extrinsics_temp << endl;
		f::WriteMat(extrinsics_temp, prj_board_rt_dst);
		W2P_rt.emplace_back(extrinsics_temp);
	}


	extrinsics_ = SolveC2Prt(W2C_rt,W2P_rt);

	/* ���������ͶӰ�ǵķ���任����C2W * W2P -> C2P */
	LOG_STRING_VAR("instrisinc_", instrisinc_);
	LOG_STRING_VAR("distortion_", distortion_);
	LOG_STRING_VAR("extrinsics_", extrinsics_);
	LOG_STRING("");

	
	/* �洢�궨���� */
	cv::imwrite(param_filename[0], instrisinc_);
	cv::imwrite(param_filename[1], extrinsics_);
	cv::imwrite(param_filename[2], distortion_);
	cv::imwrite(param_filename[3], prj_error_);
	cout << extrinsics_;

}

void projector::SetALLCalibrateParam(vector_m params)
{
	instrisinc_ = params[3];
	extrinsics_ = params[2];
	distortion_ = params[1];
	calibrate_error_ = params[0];
}

const string projector::GetName() const { return name_; }
mat projector::GetInstrisinc() const { return instrisinc_; }
mat projector::GetExtrinsics() const { return extrinsics_; }
mat projector::GetDistortion() const { return distortion_; }
int projector::GetWidth() const { return width_; }
int projector::GetHeight() const { return height_; }

/*---------------------Projector Protected---------------------*/
Point3f projector::SolveCam3dPoint(Point2f feature_2d_point, mat instrisinc, mat extrinsics)
{
	Point3f feature_3d_point;
	mat uv1 = (cv::Mat_<double>(3, 1) << feature_2d_point.x, feature_2d_point.y, 1);				// �����������

	mat R = extrinsics.colRange(0, 3);			// ��ת����
	mat t = extrinsics.colRange(3, 4);			// ƽ�ƾ���
	mat K = instrisinc;							// �ڲξ���
	double Z_c;									// �������ϵ�е�Z����(���)��δֵ֪
	double Z_w = 0;								// ��������ϵ�У���ֵ֪���궨��Ϊһ��ƽ�� -> Z_w = 0

	/* �������(�������ϵ��)��Z_c = (0+R^(-1) * t) / (R^(-1) * K^(-1) * uv1)  */
	mat left_mat = R.inv() * K.inv() * uv1;		// ��ʽ�������
	mat right_mat = R.inv() * t;
	Z_c = (Z_w + right_mat.at<double>(2, 0)) / (left_mat.at<double>(2, 0));

	/* �����������ϵ�µ���ά���� */
	mat XYZ_c = Z_c * K.inv() * uv1;

	mat XYZ_w = R.inv() * (XYZ_c - t);

	feature_3d_point.x = (float)XYZ_w.at<double>(0, 0);
	feature_3d_point.y = (float)XYZ_w.at<double>(1, 0);
	feature_3d_point.z = 0;

	return feature_3d_point;
}

/* ����任 */
mat projector::SolveC2Prt(vector_m W2C_rt, vector_m W2P_rt)			// ���ַ�ʽ�����һ�飬ƽ��ֵ����Ȩƽ��ֵ
{
	mat C2P_rt = mat::zeros(3, 4, CV_64FC1);
	double total_weight = 0;
	double weight_c = 0;
	double weight_p = 0;

	for (int i = 0; i < W2C_rt.size(); i++)
	{
		//C2P_rt = mat::zeros(3, 4, CV_64FC1);		// ���

		mat R_wc = W2C_rt[i].colRange(0, 3);			// ��ת����
		mat t_wc = W2C_rt[i].colRange(3, 4);			// ƽ�ƾ���
		mat R_wp = W2P_rt[i].colRange(0, 3);			// ��ת����
		mat t_wp = W2P_rt[i].colRange(3, 4);			// ƽ�ƾ���
		weight_c = W2C_rt[i].at<double>(1, 4);//����궨����Ȩ��
		weight_p = W2P_rt[i].at<double>(1, 4);//ͶӰ�Ǳ궨����Ȩ��
		mat R_cp = R_wp * R_wc.inv();
		cout << "��" << i + 1 << "��궨ͼ����������ת����Ϊ��" << endl << R_cp << endl;
		mat t_cp = -R_wp * R_wc.inv() * t_wc + t_wp;
		cout << "��" << i + 1 << "��궨ͼ��������ƽ�ƾ���Ϊ��" << endl << t_cp << endl;

		C2P_rt.colRange(0, 3) = C2P_rt.colRange(0, 3) + R_cp * (weight_c + weight_p);
		C2P_rt.colRange(3, 4) = C2P_rt.colRange(3, 4) + t_cp * (weight_c + weight_p);
		total_weight = total_weight + (weight_c + weight_p);
	}

	C2P_rt.colRange(0, 3) = C2P_rt.colRange(0, 3) / total_weight;
	C2P_rt.colRange(3, 4) = C2P_rt.colRange(3, 4) / total_weight;

	return C2P_rt;
}


/*  ɨ����
*
*/
bool scanner::Connect()
{
	LOG_STRING("��������ɨ���ǡ���");
	return scanner_camera.ConnectDevice() && scanner_projector.ConnectDevice();
}


/* ����ɨ���� */
bool scanner::Configure()
{
	LOG_STRING("���ڳ�ʼ��ɨ���ǡ���");
	scanner_projector.ConfigureDevice();
	scanner_camera.ConfigureDevice(1);		// TriggerModeOn
	return true;
}

/* ɨ���Ǳ궨 */
void scanner::Calibrate(CalibrateConfig& calcon)
{
	std::cout << name_ << ": ���ڱ궨����\n" << std::endl;

	/* ��������洢λ�� */
	string scanner_param_check[2];																// �������
	scanner_param_check[0] = calcon.camera_calibrate_param_folder + "//cam_instrisinc.tif";		// �������
	scanner_param_check[1] = calcon.projector_calibrate_param_folder + "//pro_instrisinc.tif";

	/* ��ȡ�豸��������� */
	if (f::FindFile(scanner_param_check[0]) && f::FindFile(scanner_param_check[1]))
	{
		scanner_camera.SetALLCalibrateParam(f::GetMats(calcon.camera_calibrate_param_folder));
		scanner_projector.SetALLCalibrateParam(f::GetMats(calcon.projector_calibrate_param_folder));
		LOG_STRING("ɨ�����ѱ궨\n");
		return;
	}
	else
	{
		LOG_STRING("ɨ����δ�궨�����ڿ�ʼ->\n");
	}

	/* ��ȡͼ��ߴ� */
	cv::Size cam_size(scanner_camera.GetWidth(), scanner_camera.GetHeight());
	cv::Size pro_size(scanner_projector.GetWidth(), scanner_projector.GetHeight());

	/* ��ȡ�����ͶӰ�Ǳ궨ͼ */
	LOG_STRING("��������궨ͼ����\n");
	vector_m cam_photos;
	vector_m pro_photos;
	bool isUseful = false;
	vector_m temp_photo;

	/* ����Ϊ�궨����Ҫ��Ͷͼģʽ */
	CalibratePatternSequenceIndex psi;
	scanner_projector.SetPatternTime(17000, 18000);
	scanner_projector.SetPatternSequenceIndex(psi.flash_index, psi.pattern_index, psi.bit_depth, psi.pattern_num);
	scanner_camera.SetPreAcquireImgNum(psi.pattern_num);
	scanner_camera.StartAcquireImages();
	for (int i = 0; i < calcon.num_calibrate; i++)
	{
		isUseful = false;
		while (!isUseful)
		{
			LOG_STRING("׼�������̸�궨�壬����ͼ���Ƿ���Ч����");
			while (!_getch());
			scanner_projector.StartDisplay();
			temp_photo = scanner_camera.GetImgs();			// ��һ���ǰ�ͼ���ڶ��������̸�
			cv::imwrite("D://Eureka//Eureka//output//calibrate//test.bmp", temp_photo[1]);
			LOG_STRING("�궨ͼ���Ƿ���Ч��0 or 1");
			std::cin >> isUseful;
		}

		mat cam_calibrate = temp_photo[0];
		mat pro_calibrate = temp_photo[1];
		cam_photos.emplace_back(cam_calibrate);
		pro_photos.emplace_back(pro_calibrate);

		/* �洢����궨ͼ */
		string name = "//" + std::to_string(i) + ".bmp";
		string path1 = "D://Eureka//Eureka//output//calibrate//camera_photos" + name;
		cv::imwrite(path1, cam_calibrate);

		/* �洢ͶӰ�Ǳ궨ͼ */
		double minVal, maxVal;
		cv::subtract(cam_calibrate, pro_calibrate, pro_calibrate);
		cv::minMaxLoc(pro_calibrate, &minVal, &maxVal);
		cv::convertScaleAbs(pro_calibrate, pro_calibrate, -255.0 / (maxVal - minVal), 255 + (255.0 + minVal) / (maxVal - minVal));
		cv::imwrite("D://Eureka//Eureka//output//calibrate//Binary0.bmp", pro_calibrate);
		string path2 = "D://Eureka//Eureka//output//calibrate//projector_photos" + name;
		cv::imwrite(path2, pro_calibrate);
	}
	scanner_projector.StopDisplay();
	scanner_camera.StopAcquireImages();
	/* ֱ�Ӷ�ȡ�궨ͼ */
	//LOG_STRING("���ڶ�ȡ(����)�궨ͼ����\n");
	//cam_photos = f::GetMats(calcon.camera_photos_folder);
	//pro_photos = f::GetMats(calcon.projector_photos_folder);
	
	/* �궨��� */
	LOG_STRING("��ʼ����궨->");
	vector_m W2C_rt;					// ��������ϵ���������ϵ�ķ���任����
	W2C_rt = scanner_camera.Calibrate(cam_photos, calcon);

	/* �궨ͶӰ�� */
	LOG_STRING("��ʼͶӰ�Ǳ궨->");
	scanner_projector.Calibrate(pro_photos, W2C_rt, scanner_camera, calcon);

	std::cout << name_ << ": �궨��ɣ�\n" << std::endl;
}

/* ���������ռ� */
void scanner::SetWorkSpace()
{

	bool workspaceIsOK;
	/* ����ͶӰ�ǹ������� */
	LOG_STRING("ƥ��ͶӰ�ǹ�������");
	CheckPatternSequenceIndex psi;										// ����ͶӰ���� = ���̸�
	scanner_projector.SetPatternTime(8000000, 8000000);				// �����ع�ʱ��
	scanner_projector.SetPatternSequenceIndex(psi.flash_index, psi.pattern_index, psi.bit_depth, psi.pattern_num);
	for (int i = 0; i < 3; i++)
	{
		if (!scanner_projector.StartDisplay()) { break; }
	}
	Sleep(8000);
	scanner_projector.StopDisplay();


	LOG_STRING("�����������͹�Ȧ");
	scanner_projector.SetPatternTime(17000, 18000);		          // �����ع�ʱ��
	scanner_camera.SetPreAcquireImgNum(psi.pattern_num);
	scanner_camera.StartAcquireImages();
	while (1)
	{
		if (!scanner_projector.StartDisplay()) { break; }
		mat test_photo = scanner_camera.GetImgs()[0];			  // ����ģʽ����
		cv::namedWindow("test_workspace", 0);
		cv::imshow("test_workspace", test_photo);
		cv::waitKey();
		LOG_STRING("���ͼ���Ƿ�����(0/1)");
		std::cin >> workspaceIsOK;
		if (workspaceIsOK) { break; }
	}
	scanner_projector.StopDisplay();
	scanner_camera.StopAcquireImages();
}

/* ��ʼɨ�� */
vector_m scanner::StartScan()
{
	if (scan_times_ == 0)			// ͶӰ��ֻ��Ҫ����һ�Σ�ͬʱ����Ҫ���ö�Ӧ���������ͼ������
	{
		ScanPatternSequenceIndex psi;
		scanner_projector.SetPatternTime(17000,18000);
		scanner_projector.SetPatternSequenceIndex(psi.flash_index, psi.pattern_index, psi.bit_depth, psi.pattern_num);
		scanner_camera.SetPreAcquireImgNum(psi.pattern_num);
	}

	clock_t scan_start = clock();
	scanner_camera.StartAcquireImages();			// ����ⴥ��ģʽ�£��ȿ����������ͶӰ������ֻ��Ҫ����һ�Σ������ͼģʽ�£���ͶӰ���ٿ������
	scanner_projector.StartDisplay();
	vector_m scan_photos = scanner_camera.GetImgs();
	clock_t scan_end = clock();

	running_time_ = double(scan_end - scan_start);

	scan_times_++;

	return scan_photos;
}

/* ֹͣɨ�� */
void scanner::StopScan()
{
	scanner_camera.StopAcquireImages();
	scanner_projector.StopDisplay();
}

/* �Ͽ����� */
void scanner::Disconnect()
{
	scanner_camera.DisconnectDevice();
	scanner_projector.DisconnectDevice();
}

const string scanner::GetName() const { return name_; }
const double scanner::GetRunningTime() const { return running_time_; }