#include "hardware.h"
#include "triangulation.h"


/* 成功 */
mat Triangulation(scanner& scanner_user, mat& photos_decoded, mat& mask)
{
	std::cout << scanner_user.GetName() << ": 正在解算点云\n" << std::endl;

	mat cam_k = scanner_user.scanner_camera.GetInstrisinc();
	//mat cam_Rt = scanner_user.scanner_camera.GetExtrinsics();					// Rwc = [R|t] = [I|0] 选择相机坐标系为世界坐标系,可忽略相机外参
	mat cam_d = scanner_user.scanner_camera.GetDistortion();
	mat pro_k = scanner_user.scanner_projector.GetInstrisinc();
	mat pro_Rt = scanner_user.scanner_projector.GetExtrinsics();				// Rwp = Rcp = [R|t]
	mat pro_d = scanner_user.scanner_projector.GetDistortion();
	int width = scanner_user.scanner_camera.GetWidth();
	int height = scanner_user.scanner_camera.GetHeight();

	mat p_cam_XYZ(height, width, CV_64FC3);	
	p_cam_XYZ.forEach<cv::Vec3d>([&mask, &width, &photos_decoded, &cam_k, &pro_k, &pro_Rt](cv::Vec3d& val, const int* pos) 
	{
		if (mask.ptr<uchar>(pos[0])[pos[1]])
		{
			const double p_cam_u_f = (pos[1] - cam_k.ptr<double>(0)[2]) / cam_k.ptr<double>(0)[0];		// 还原相机像素坐标偏置并除以焦距 u-x-col-pos[1]
			const double p_cam_v_f = (pos[0] - cam_k.ptr<double>(1)[2]) / cam_k.ptr<double>(1)[1];

			const double p_pro_u_f = (photos_decoded.ptr<float>(pos[0])[pos[1]] - pro_k.ptr<double>(0)[2]) / pro_k.ptr<double>(0)[0];		// 还原投影仪像素偏置(x方向)

			double molecular = pro_Rt.ptr<double>(0)[3] - pro_Rt.ptr<double>(2)[3] * p_pro_u_f;		        // 分子：t1 - t3 * s

			double denominator1 = pro_Rt.ptr<double>(2)[0] * p_cam_u_f * p_pro_u_f	// 分母：r31 * uc * s / fcx + r32 * uc * s / fcy + r33 * s;
				+ pro_Rt.ptr<double>(2)[1] * p_cam_v_f * p_pro_u_f
				+ pro_Rt.ptr<double>(2)[2] * p_pro_u_f;
			double denominator2 = pro_Rt.ptr<double>(0)[0] * p_cam_u_f				// 分母：r11 * uc / fcx + r12 * uc / fcy + r13;
				+ pro_Rt.ptr<double>(0)[1] * p_cam_v_f
				+ pro_Rt.ptr<double>(0)[2];
			double denominator = denominator1 - denominator2;

			val[2] = molecular / denominator;										// Zc = molecular / denominator;

			val[0] = p_cam_u_f * val[2];
			val[1] = p_cam_v_f * val[2];
		}

		else
		{
			val[0] = 0;	val[1] = 0;	val[2] = 0;
		}

		}
	);

	/* 点云后处理 */
	// 异常值，极大程度大于均值的点【为什么会有这样的点，哪个环节出现的问题】
	p_cam_XYZ.forEach<cv::Vec3d>([](cv::Vec3d& val, const int* pos) {
		if (val[2] >= 800 || val[2] <= 0)		// 剔除大于正常工作距离的点，以及不合理的点【为什么会产生不合理的点】
		{
			val[0] = 0;	val[1] = 0;	val[2] = 0;
		}
		});

	return p_cam_XYZ;
}


mat udTriangulation(scanner& scanner_user, mat& photos_decoded, mat& mask)
{
	std::cout << scanner_user.GetName() << ": 正在解算点云\n" << std::endl;

	mat cam_k = scanner_user.scanner_camera.GetInstrisinc();
	//mat cam_Rt = scanner_user.scanner_camera.GetExtrinsics();					// Rwc = [R|t] = [I|0] 选择相机坐标系为世界坐标系,可忽略相机外参
	mat cam_d = scanner_user.scanner_camera.GetDistortion();
	mat pro_k = scanner_user.scanner_projector.GetInstrisinc();
	mat pro_Rt = scanner_user.scanner_projector.GetExtrinsics();				// Rwp = Rcp = [R|t]
	mat pro_d = scanner_user.scanner_projector.GetDistortion();
	int width = scanner_user.scanner_camera.GetWidth();
	int height = scanner_user.scanner_camera.GetHeight();
	cout << cam_k << endl;
	cout << cam_d << endl;
	cout << pro_k << endl;
	cout << pro_d << endl;
	cout << pro_Rt << endl;
	mat p_cam_XYZ(height, width, CV_64FC3);
	p_cam_XYZ.forEach<cv::Vec3d>([&mask, &width, &photos_decoded, &cam_k, &pro_k, &pro_Rt, &pro_d](cv::Vec3d& val, const int* pos)
	{
		if (mask.ptr<uchar>(pos[0])[pos[1]])
		{
			const double p_cam_u_f = (pos[1] - cam_k.ptr<double>(0)[2]) / cam_k.ptr<double>(0)[0];		// 还原相机像素坐标偏置并除以焦距 u-x-col-pos[1]
			const double p_cam_v_f = (pos[0] - cam_k.ptr<double>(1)[2]) / cam_k.ptr<double>(1)[1];

			const double p_pro_u_f = (photos_decoded.ptr<float>(pos[0])[pos[1]] - pro_k.ptr<double>(0)[2]) / pro_k.ptr<double>(0)[0];		// 还原投影仪像素偏置(x方向)

			cv::Vec3d xyz_p;
			cv::Vec2d xy_p;
			double ud_p_pro_u_f = p_pro_u_f;

			for (int j = 0; j < 3; j++)
			{
				
				double molecular = pro_Rt.ptr<double>(0)[3] - pro_Rt.ptr<double>(2)[3] * ud_p_pro_u_f;		        // 分子：t1 - t3 * s

				double denominator1 = pro_Rt.ptr<double>(2)[0] * p_cam_u_f * ud_p_pro_u_f	// 分母：r31 * uc * s / fcx + r32 * uc * s / fcy + r33 * s;
					+ pro_Rt.ptr<double>(2)[1] * p_cam_v_f * ud_p_pro_u_f
					+ pro_Rt.ptr<double>(2)[2] * ud_p_pro_u_f;
				double denominator2 = pro_Rt.ptr<double>(0)[0] * p_cam_u_f				// 分母：r11 * uc / fcx + r12 * uc / fcy + r13;
					+ pro_Rt.ptr<double>(0)[1] * p_cam_v_f
					+ pro_Rt.ptr<double>(0)[2];
				double denominator = denominator1 - denominator2;

				val[2] = molecular / denominator;										// Zc = molecular / denominator;

				val[0] = p_cam_u_f * val[2];
				val[1] = p_cam_v_f * val[2];

				//计算投影仪三维坐标点
				xyz_p[0] = pro_Rt.ptr<double>(0)[0] * val[0] + pro_Rt.ptr<double>(0)[1] * val[1] + pro_Rt.ptr<double>(0)[2] * val[2] + pro_Rt.ptr<double>(0)[3];
				xyz_p[1] = pro_Rt.ptr<double>(1)[0] * val[0] + pro_Rt.ptr<double>(1)[1] * val[1] + pro_Rt.ptr<double>(1)[2] * val[2] + pro_Rt.ptr<double>(1)[3];
				xyz_p[2] = pro_Rt.ptr<double>(2)[0] * val[0] + pro_Rt.ptr<double>(2)[1] * val[1] + pro_Rt.ptr<double>(2)[2] * val[2] + pro_Rt.ptr<double>(2)[3];

				//计算投影仪像素坐标系坐标点
				xy_p[0] = p_pro_u_f;
				xy_p[0] = xy_p[0] * pro_k.ptr<double>(0)[0] + pro_k.ptr<double>(0)[2];
				xy_p[1] = xyz_p[1] / xyz_p[2];//横坐标不变，纵坐标替代
				xy_p[1] = xy_p[1] * pro_k.ptr<double>(1)[1] + pro_k.ptr<double>(1)[2];

				//投影仪去畸变
				vector<cv::Point2f> P;
				vector<cv::Point2f>	ud_P;
				P.push_back(cv::Point2f(xy_p[0], xy_p[1]));
				ud_P.push_back(cv::Point2f(0, 0));
				cv::undistortPoints(P, ud_P, pro_k, pro_d, cv::noArray(), pro_k);//归一化坐标系下会出错
                //cout<< ud_P[0].x <<endl;
				ud_P[0].x= (ud_P[0].x - pro_k.ptr<double>(0)[2]) / pro_k.ptr<double>(0)[0];
				ud_p_pro_u_f = ud_P[0].x;
			}
		}

		else
		{
			val[0] = 0;	val[1] = 0;	val[2] = 0;
		}

	}
	);

	///* 点云后处理 */
	//// 异常值，极大程度大于均值的点【为什么会有这样的点，哪个环节出现的问题】
	//p_cam_XYZ.forEach<cv::Vec3d>([](cv::Vec3d& val, const int* pos) {
	//	if (val[2] >= 500 || val[2] <= 0)		// 剔除大于正常工作距离的点，以及不合理的点【为什么会产生不合理的点】
	//	{
	//		val[0] = 0;	val[1] = 0;	val[2] = 0;
	//	}
	//});

	return p_cam_XYZ;

}