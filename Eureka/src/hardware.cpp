#include "hardware.h"
#include<math.h>




/*  相机  */
vector_m camera::Calibrate(vector_m& photos, CalibrateConfig& calcon)
{
	string points_dst;
	/* 相机参数存储位置 */
	string param_filename[4];							// 相机参数
	param_filename[0] = calcon.camera_calibrate_param_folder + "//cam_instrisinc.tif";
	param_filename[1] = calcon.camera_calibrate_param_folder + "//cam_extrinsics.tif";
	param_filename[2] = calcon.camera_calibrate_param_folder + "//cam_distortion.tif";
	param_filename[3] = calcon.camera_calibrate_param_folder + "//cam_calibrate_error.tif";

	/* 获取图像尺寸 */
	cv::Size cam_size(width_, height_);

	/* 2d(O_uv): 提取标定图中各特征点及其二维信息(提取角点) (图像像素坐标系) */
	LOG_STRING("正在提取特征点……\n");
	v_vector_2f total_feature_2d_points;												// 标定板上的二维角点坐标【考虑预分配地址】
	cv::Size points_num(calcon.num_board_point[0], calcon.num_board_point[1]);			//【能否自动获取行列角点个数】
	for (int i = 0; i < photos.size(); i++)
	{
		vector_2f feature_2d_points;
		cv::SimpleBlobDetector::Params params;
		params.blobColor =0;
		params.maxArea = 5000;
		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
		//检测圆心，顺序为左上到右下
		//if (!cv::findCirclesGrid(photos[i], points_num, feature_2d_points, cv::CALIB_CB_SYMMETRIC_GRID| cv::CALIB_CB_CLUSTERING,detector))
		if (!cv::findCirclesGrid(photos[i], points_num, feature_2d_points, cv::CALIB_CB_SYMMETRIC_GRID))
		//检测角点，顺序从左上到右下
		//if (!cv::findChessboardCornersSB(photos[i], points_num, feature_2d_points, cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY))			
		{
			LOG_STRING_VAR("特征点提取失败", i);
		}
		else
		{
			if (i == 0) { std::cout << "特征点提取成功:\t"; }
			std::cout << i << "\t";
			if (i == photos.size() - 1) { std::cout << std::endl; }

			total_feature_2d_points.emplace_back(feature_2d_points);

			points_dst = "D://Eureka//Eureka//output//calibrate//points//feature_cam" + std::to_string(i) + ".txt";
			f::Write2dPointsToFile(feature_2d_points, points_dst);

			//在标定图片上标记提取到的角点
			cv::drawChessboardCorners(photos[i], points_num, feature_2d_points, true); //用于在图片中标记角点
			string dst;
			dst= "D://Eureka//Eureka//output//calibrate//camera_photos//角点提取//" +std::to_string(i) + ".bmp";
			cv::imwrite(dst, photos[i]);
		}
	}

	/* 3d(O_w): 计算所有二维角点对应的三维信息，Zw=0 (世界坐标系) */
	LOG_STRING("计算对应的三维坐标……");
	v_vector_3f total_feature_3d_points;						// 标定板上的三维角点坐标
	vector_3f  feature_3d_points;
	for (int i = 0; i < total_feature_2d_points.size(); i++)
	{
		if (i == 0)
		{
			Point3f feature_3d_point;

			//正序计算
			for (int y = 1; y <= points_num.height; y++)			// 每张图中角点总数
			{
				for (int x = 1; x <= points_num.width; x++)
				{
					feature_3d_point.x = x * calcon.distance_board_point[0];				// 当前二维角点对应的三维点坐标
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

	/* opencv标定 */
	extrinsics_ = mat::zeros(3, 4, CV_64FC1);		// 相机坐标系作为世界坐标系，其外参为0矩阵
	vector_m rvecs;
	vector_m tvecs;
	mat extrinsics_m;
	
	double calibrate_error =cv::calibrateCamera(total_feature_3d_points, total_feature_2d_points, cam_size, instrisinc_, distortion_, rvecs, tvecs, cv::CALIB_FIX_K3| cv::CALIB_ZERO_TANGENT_DIST);//CALIB_ZERO_TANGENT_DIST可选
	cout << calibrate_error << endl;
	/*计算重投影误差*/
	//初始化角点数量
	LOG_STRING("标定完成，开始评价标定结果");
	int i = 0;
	int num = int(total_feature_3d_points.size());
	vector<int> point_counts;  // 每幅图像中角点的数量
	for (i = 0; i < num; i++)
	{
		point_counts.push_back(points_num.width * points_num.height);
	}

	vector<Point2f> total_feature_2d_points2;//保存重新计算得到的投影点
	vector<double> cam_errors;	// 标定误差 (重投影误差、距离误差、……)	
	double cam_error_ = 0;
	int totalPoints = 0;
	double totalErr = 0;

	for (i = 0; i < num; i++)
	{
		vector<Point3f> tempPointSet = total_feature_3d_points[i];
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		projectPoints(tempPointSet, rvecs[i], tvecs[i],
			instrisinc_, distortion_, total_feature_2d_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
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
		std::cout << "第" << i + 1 << "幅图像的平均误差：" << cam_error_ << "像素" << endl;
	}

	std::cout << "总体平均误差：" << totalErr / num << "像素" << endl;
	points_dst = "D://Eureka//Eureka//output//calibrate//cam_error.txt";
	f::WriteErrorToFile(totalErr / num, points_dst);

	/* 标定参数转换 */
	vector_m W2C_rt;										// 【考虑实现预分配内存】
	string cam_board_rt_dst;
	for (int i = 0; i < total_feature_2d_points.size(); i++)
	{
		mat extrinsics_temp = mat::zeros(3, 5, CV_64FC1);

		cv::Rodrigues(rvecs[i], rvecs[i]);		// 旋转向量转换到旋转矩阵
		extrinsics_temp.colRange(0, 3) = extrinsics_temp.colRange(0, 3) + rvecs[i];
		extrinsics_temp.colRange(3, 4) = extrinsics_temp.colRange(3, 4) + tvecs[i];
		extrinsics_temp.at<double>(0,4)= cam_errors[i];
		extrinsics_temp.at<double>(1, 4) = 1/cam_errors[i];
		cam_board_rt_dst = "D://Eureka//Eureka//output//calibrate//cam_board//cam_board_rt"+std::to_string(i) + ".txt";
		//cout << extrinsics_temp << endl;
		f::WriteMat(extrinsics_temp, cam_board_rt_dst);
		W2C_rt.emplace_back(extrinsics_temp);
	}
	/* 评价标定精度 */
	LOG_STRING_VAR("camera_calibrated.GetInstrisinc()", instrisinc_);
	LOG_STRING_VAR("camera_calibrated.GetDistortion()", distortion_);
	LOG_STRING("");

	/* 存储标定结果 */
	cv::imwrite(param_filename[0], instrisinc_);
	cv::imwrite(param_filename[1], extrinsics_);
	cv::imwrite(param_filename[2], distortion_);
	cv::imwrite(param_filename[3], totalErr / num);

	return W2C_rt;
}

void camera::SetALLCalibrateParam(vector_m params)
{
	instrisinc_ = params[3];			// 字典序：instrisinc < extrinsics < distortion < calibrate_error -> 3,2,1,0
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



/*  投影仪
*
*/
void projector::Calibrate(vector_m& photos, vector_m& W2C_rt, camera& camera_calibrated, CalibrateConfig& calcon)
{
	string points_dst;
	/* 投影仪参数存储位置 */
	string param_filename[4];							// 投影仪参数
	param_filename[0] = calcon.projector_calibrate_param_folder + "//pro_instrisinc.tif";
	param_filename[1] = calcon.projector_calibrate_param_folder + "//pro_extrinsics.tif";
	param_filename[2] = calcon.projector_calibrate_param_folder + "//pro_distortion.tif";
	param_filename[3] = calcon.projector_calibrate_param_folder + "//pro_calibrate_error.tif";

	/* 获取棋盘格投影图尺寸 */
	cv::Size pro_size(width_, height_);

	//for (int i = 0; i < photos.size(); i++)
	//{
	//	cv::threshold(photos[i], photos[i], 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	//}
	

	//使用相机参数对投影仪标定图案去畸变
	vector_m ud_photos;
	for (int i = 0; i < photos.size(); i++)
	{
		mat ud_p;
		cv::undistort(photos[i], ud_p, camera_calibrated.GetInstrisinc(), camera_calibrated.GetDistortion());
		string dst;
		dst = "D://Eureka//Eureka//output//calibrate//projector_photos//去畸变后//" + std::to_string(i) + ".bmp";
		cv::imwrite(dst, ud_p);
		ud_photos.emplace_back(ud_p);
	}


	/* 寻找相机拍摄图片中的二维特征点 */
	LOG_STRING("正在提取特征点……\n");
	v_vector_2f total_cam_feature_2d_points;
	vector_2f cam_feature_2d_points;
	Point2f cam_feature_2d_point;
	cv::Size points_num(calcon.num_projector_point[0], calcon.num_projector_point[1]);
	for (int i = 0; i < photos.size(); i++)
	{
		//检测角点
		//if (!cv::findChessboardCorners(ud_photos[i], points_num, cam_feature_2d_points, cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_FILTER_QUADS))// 二维坐标逆序排列的，从右下到左上
		if (!cv::findChessboardCornersSB(ud_photos[i], points_num, cam_feature_2d_points, cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY))			//二维坐标正序排列，从左上到右下
		{
			LOG_STRING_VAR("特征点提取失败", i);
			W2C_rt.erase(W2C_rt.begin() + i);					// 剔除(擦除)对应的RT矩阵
		}
		else
		{
			if (i == 0) { std::cout << "特征点提取成功:\t"; }
			std::cout << i << "\t";
			if (i == photos.size() - 1) { std::cout << std::endl; }
			//对提取角点进行亚像素级精确化;
			//cv::cornerSubPix(ud_photos[i], cam_feature_2d_points, cv::Size(11, 11),cv::Size(-1, -1), cv::TermCriteria(0, 30, 0.1));
			//记录角点数据
			points_dst = "D://Eureka//Eureka//output//calibrate//points//feature_pro" + std::to_string(i) + ".txt";
			f::Write2dPointsToFile(cam_feature_2d_points, points_dst);
			total_cam_feature_2d_points.emplace_back(cam_feature_2d_points);// 顺序：自上而下，自左向右
			//在标定图片上标记提取到的角点
			cv::drawChessboardCorners(ud_photos[i], points_num, cam_feature_2d_points, true); //用于在图片中标记角点
			string dst;
			dst = "D://Eureka//Eureka//output//calibrate//projector_photos//角点提取//" + std::to_string(i) + ".bmp";
			cv::imwrite(dst, ud_photos[i]);
		}
	}
	if (W2C_rt.size() != total_cam_feature_2d_points.size())		// 安全保护
	{
		LOG_STRING("错误，有提取特征点失败的图像，但未成功擦除对应的外参矩阵");

		return;
	}
	else if (W2C_rt.size() < 4)
	{
		LOG_STRING("错误，有提取特征点失败的图像，但未成功擦除对应的外参矩阵");
		return;
	}

	/* 使用相机的内外参数将图像坐标系下的二维特征点转换为世界坐标系下的三维特征点 */
	v_vector_3f total_cam_feature_3d_points;
	vector_3f cam_feature_3d_points;
	Point3f cam_feature_3d_point;
	for (int i = 0; i < total_cam_feature_2d_points.size(); i++)			// n张“有效标定图”
	{
		cam_feature_3d_points = {};											// 清空	【为什么这里不存在“emplace_back问题”】
		cam_feature_2d_points = total_cam_feature_2d_points[i];
		for (int j = 0; j < points_num.area(); j++)							// 特征点总数
		{
			cam_feature_2d_point = cam_feature_2d_points[j];				// 当前的一个二维特征点
			cam_feature_3d_point = SolveCam3dPoint(cam_feature_2d_point, camera_calibrated.GetInstrisinc(), W2C_rt[i]);	    // 计算二维特征点(相机像素坐标系)对应的三维特征点(相机坐标系)
			cam_feature_3d_points.emplace_back(cam_feature_3d_point);

			points_dst = "D://Eureka//Eureka//output//calibrate//points//board_pro" + std::to_string(i) + ".txt";
			f::Write3dPointsToFile(cam_feature_3d_points, points_dst);
		}
		total_cam_feature_3d_points.emplace_back(cam_feature_3d_points);
	}

	/* 计算棋盘格投影仪图中特征点(角点)的二维坐标 */
	/* 3d(O_w): 计算所有二维角点对应的三维信息，Zw=0 (世界坐标系) */
	LOG_STRING("计算对应的三维坐标……");
	v_vector_2f total_pro_feature_2d_points;									// 投影仪的二维角点坐标
	vector_2f  pro_feature_2d_points;
	for (int i = 0; i < total_cam_feature_2d_points.size(); i++)				// 有效标定图
	{
		if (i == 0)
		{
			
			Point2f feature_2d_point;											// 当前二维角点对应的二维坐标
			
			//倒序计算
			//for (int y = points_num.height; y >= 1; y--)			// 每张图中角点总数
			//{
			//	for (int x = points_num.width; x >= 1;  x--)
			//	{
			//		feature_2d_point.x = 96 + x * calcon.distance_projector_point[0];
			//		feature_2d_point.y = 60 + y * calcon.distance_projector_point[1];
			//		pro_feature_2d_points.emplace_back(feature_2d_point);
			//	}
			//}

			//正序计算
			for (int y = 1; y <= points_num.height; y++)			// 每张图中角点总数
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

	/* opencv 标定 */
	vector_m rvecs;
	vector_m tvecs;
	double calibrate_error_= cv::calibrateCamera(total_cam_feature_3d_points, total_pro_feature_2d_points, pro_size, instrisinc_, distortion_, rvecs, tvecs, cv::CALIB_FIX_K3| cv::CALIB_ZERO_TANGENT_DIST);//CALIB_ZERO_TANGENT_DIST可选
	cout << calibrate_error_ << endl;

	/*计算重投影误差*/
	//初始化角点数量
	LOG_STRING("标定完成，开始评价标定结果");
	int i = 0;
	int num = int(total_cam_feature_3d_points.size());
	vector<int> point_counts;  // 每幅图像中角点的数量
	for (i = 0; i < num; i++)
	{
		point_counts.push_back(points_num.width * points_num.height);
	}

	vector<Point2f> total_pro_feature_2d_points2;//保存重新计算得到的投影点
	vector<double> prj_errors;
	double prj_error_;	// 标定误差 (重投影误差、距离误差、……)	
	int totalPoints = 0;
	double totalErr = 0;

	for (i = 0; i < num; i++)
	{
		vector<Point3f> tempPointSet = total_cam_feature_3d_points[i];
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		projectPoints(tempPointSet, rvecs[i], tvecs[i],
			instrisinc_, distortion_, total_pro_feature_2d_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
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
		std::cout << "第" << i + 1 << "幅图像的平均误差：" << prj_error_ << "像素" << endl;
	}

	std::cout << "总体平均误差：" << totalErr / num << "像素" << endl;
	points_dst = "D://Eureka//Eureka//output//calibrate//pro_error.txt";
	f::WriteErrorToFile(totalErr / num, points_dst);


	/* 标定参数转换 */
	vector_m W2P_rt;	// 【考虑实现预分配内存】
	string prj_board_rt_dst;
	for (int i = 0; i < total_cam_feature_2d_points.size(); i++)
	{
		mat extrinsics_temp = mat::zeros(3, 5, CV_64FC1);			// 【非常重要的问题：vector的内存分配方法(什么时候会出现这种问题？)】

		cv::Rodrigues(rvecs[i], rvecs[i]);		// 旋转向量转换到旋转矩阵
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

	/* 计算相机到投影仪的仿射变换矩阵：C2W * W2P -> C2P */
	LOG_STRING_VAR("instrisinc_", instrisinc_);
	LOG_STRING_VAR("distortion_", distortion_);
	LOG_STRING_VAR("extrinsics_", extrinsics_);
	LOG_STRING("");

	
	/* 存储标定参数 */
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
	mat uv1 = (cv::Mat_<double>(3, 1) << feature_2d_point.x, feature_2d_point.y, 1);				// 像素齐次坐标

	mat R = extrinsics.colRange(0, 3);			// 旋转矩阵
	mat t = extrinsics.colRange(3, 4);			// 平移矩阵
	mat K = instrisinc;							// 内参矩阵
	double Z_c;									// 相机坐标系中的Z坐标(深度)，未知值
	double Z_w = 0;								// 世界坐标系中，已知值，标定板为一个平面 -> Z_w = 0

	/* 计算深度(相机坐标系下)：Z_c = (0+R^(-1) * t) / (R^(-1) * K^(-1) * uv1)  */
	mat left_mat = R.inv() * K.inv() * uv1;		// 算式的左矩阵
	mat right_mat = R.inv() * t;
	Z_c = (Z_w + right_mat.at<double>(2, 0)) / (left_mat.at<double>(2, 0));

	/* 计算相机坐标系下的三维坐标 */
	mat XYZ_c = Z_c * K.inv() * uv1;

	mat XYZ_w = R.inv() * (XYZ_c - t);

	feature_3d_point.x = (float)XYZ_w.at<double>(0, 0);
	feature_3d_point.y = (float)XYZ_w.at<double>(1, 0);
	feature_3d_point.z = 0;

	return feature_3d_point;
}

/* 坐标变换 */
mat projector::SolveC2Prt(vector_m W2C_rt, vector_m W2P_rt)			// 三种方式：最后一组，平均值，加权平均值
{
	mat C2P_rt = mat::zeros(3, 4, CV_64FC1);
	double total_weight = 0;
	double weight_c = 0;
	double weight_p = 0;

	for (int i = 0; i < W2C_rt.size(); i++)
	{
		//C2P_rt = mat::zeros(3, 4, CV_64FC1);		// 清空

		mat R_wc = W2C_rt[i].colRange(0, 3);			// 旋转矩阵
		mat t_wc = W2C_rt[i].colRange(3, 4);			// 平移矩阵
		mat R_wp = W2P_rt[i].colRange(0, 3);			// 旋转矩阵
		mat t_wp = W2P_rt[i].colRange(3, 4);			// 平移矩阵
		weight_c = W2C_rt[i].at<double>(1, 4);//相机标定参数权重
		weight_p = W2P_rt[i].at<double>(1, 4);//投影仪标定参数权重
		mat R_cp = R_wp * R_wc.inv();
		cout << "第" << i + 1 << "组标定图像计算出的旋转矩阵为：" << endl << R_cp << endl;
		mat t_cp = -R_wp * R_wc.inv() * t_wc + t_wp;
		cout << "第" << i + 1 << "组标定图像计算出的平移矩阵为：" << endl << t_cp << endl;

		C2P_rt.colRange(0, 3) = C2P_rt.colRange(0, 3) + R_cp * (weight_c + weight_p);
		C2P_rt.colRange(3, 4) = C2P_rt.colRange(3, 4) + t_cp * (weight_c + weight_p);
		total_weight = total_weight + (weight_c + weight_p);
	}

	C2P_rt.colRange(0, 3) = C2P_rt.colRange(0, 3) / total_weight;
	C2P_rt.colRange(3, 4) = C2P_rt.colRange(3, 4) / total_weight;

	return C2P_rt;
}


/*  扫描仪
*
*/
bool scanner::Connect()
{
	LOG_STRING("正在连接扫描仪……");
	return scanner_camera.ConnectDevice() && scanner_projector.ConnectDevice();
}


/* 配置扫描仪 */
bool scanner::Configure()
{
	LOG_STRING("正在初始化扫描仪……");
	scanner_projector.ConfigureDevice();
	scanner_camera.ConfigureDevice(1);		// TriggerModeOn
	return true;
}

/* 扫描仪标定 */
void scanner::Calibrate(CalibrateConfig& calcon)
{
	std::cout << name_ << ": 正在标定……\n" << std::endl;

	/* 相机参数存储位置 */
	string scanner_param_check[2];																// 相机参数
	scanner_param_check[0] = calcon.camera_calibrate_param_folder + "//cam_instrisinc.tif";		// 相机参数
	scanner_param_check[1] = calcon.projector_calibrate_param_folder + "//pro_instrisinc.tif";

	/* 读取设备的内外参数 */
	if (f::FindFile(scanner_param_check[0]) && f::FindFile(scanner_param_check[1]))
	{
		scanner_camera.SetALLCalibrateParam(f::GetMats(calcon.camera_calibrate_param_folder));
		scanner_projector.SetALLCalibrateParam(f::GetMats(calcon.projector_calibrate_param_folder));
		LOG_STRING("扫描仪已标定\n");
		return;
	}
	else
	{
		LOG_STRING("扫描仪未标定，现在开始->\n");
	}

	/* 获取图像尺寸 */
	cv::Size cam_size(scanner_camera.GetWidth(), scanner_camera.GetHeight());
	cv::Size pro_size(scanner_projector.GetWidth(), scanner_projector.GetHeight());

	/* 获取相机、投影仪标定图 */
	LOG_STRING("正在拍摄标定图……\n");
	vector_m cam_photos;
	vector_m pro_photos;
	bool isUseful = false;
	vector_m temp_photo;

	/* 配置为标定所需要的投图模式 */
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
			LOG_STRING("准备好棋盘格标定板，测试图像是否有效……");
			while (!_getch());
			scanner_projector.StartDisplay();
			temp_photo = scanner_camera.GetImgs();			// 第一张是白图，第二张是棋盘格
			cv::imwrite("D://Eureka//Eureka//output//calibrate//test.bmp", temp_photo[1]);
			LOG_STRING("标定图像是否有效：0 or 1");
			std::cin >> isUseful;
		}

		mat cam_calibrate = temp_photo[0];
		mat pro_calibrate = temp_photo[1];
		cam_photos.emplace_back(cam_calibrate);
		pro_photos.emplace_back(pro_calibrate);

		/* 存储相机标定图 */
		string name = "//" + std::to_string(i) + ".bmp";
		string path1 = "D://Eureka//Eureka//output//calibrate//camera_photos" + name;
		cv::imwrite(path1, cam_calibrate);

		/* 存储投影仪标定图 */
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
	/* 直接读取标定图 */
	//LOG_STRING("正在读取(拍摄)标定图……\n");
	//cam_photos = f::GetMats(calcon.camera_photos_folder);
	//pro_photos = f::GetMats(calcon.projector_photos_folder);
	
	/* 标定相机 */
	LOG_STRING("开始相机标定->");
	vector_m W2C_rt;					// 世界坐标系到相机坐标系的仿射变换矩阵
	W2C_rt = scanner_camera.Calibrate(cam_photos, calcon);

	/* 标定投影仪 */
	LOG_STRING("开始投影仪标定->");
	scanner_projector.Calibrate(pro_photos, W2C_rt, scanner_camera, calcon);

	std::cout << name_ << ": 标定完成！\n" << std::endl;
}

/* 调整工作空间 */
void scanner::SetWorkSpace()
{

	bool workspaceIsOK;
	/* 测试投影仪工作距离 */
	LOG_STRING("匹配投影仪工作距离");
	CheckPatternSequenceIndex psi;										// 设置投影序列 = 棋盘格
	scanner_projector.SetPatternTime(8000000, 8000000);				// 设置曝光时间
	scanner_projector.SetPatternSequenceIndex(psi.flash_index, psi.pattern_index, psi.bit_depth, psi.pattern_num);
	for (int i = 0; i < 3; i++)
	{
		if (!scanner_projector.StartDisplay()) { break; }
	}
	Sleep(8000);
	scanner_projector.StopDisplay();


	LOG_STRING("调整相机焦距和光圈");
	scanner_projector.SetPatternTime(17000, 18000);		          // 更新曝光时间
	scanner_camera.SetPreAcquireImgNum(psi.pattern_num);
	scanner_camera.StartAcquireImages();
	while (1)
	{
		if (!scanner_projector.StartDisplay()) { break; }
		mat test_photo = scanner_camera.GetImgs()[0];			  // 触发模式拍照
		cv::namedWindow("test_workspace", 0);
		cv::imshow("test_workspace", test_photo);
		cv::waitKey();
		LOG_STRING("检查图像是否清晰(0/1)");
		std::cin >> workspaceIsOK;
		if (workspaceIsOK) { break; }
	}
	scanner_projector.StopDisplay();
	scanner_camera.StopAcquireImages();
}

/* 开始扫描 */
vector_m scanner::StartScan()
{
	if (scan_times_ == 0)			// 投影仪只需要设置一次，同时还需要设置对应的相机拍摄图像张数
	{
		ScanPatternSequenceIndex psi;
		scanner_projector.SetPatternTime(17000,18000);
		scanner_projector.SetPatternSequenceIndex(psi.flash_index, psi.pattern_index, psi.bit_depth, psi.pattern_num);
		scanner_camera.SetPreAcquireImgNum(psi.pattern_num);
	}

	clock_t scan_start = clock();
	scanner_camera.StartAcquireImages();			// 相机外触发模式下，先开启相机，再投影，而且只需要开启一次；相机单图模式下，先投影，再开启相机
	scanner_projector.StartDisplay();
	vector_m scan_photos = scanner_camera.GetImgs();
	clock_t scan_end = clock();

	running_time_ = double(scan_end - scan_start);

	scan_times_++;

	return scan_photos;
}

/* 停止扫描 */
void scanner::StopScan()
{
	scanner_camera.StopAcquireImages();
	scanner_projector.StopDisplay();
}

/* 断开连接 */
void scanner::Disconnect()
{
	scanner_camera.DisconnectDevice();
	scanner_projector.DisconnectDevice();
}

const string scanner::GetName() const { return name_; }
const double scanner::GetRunningTime() const { return running_time_; }