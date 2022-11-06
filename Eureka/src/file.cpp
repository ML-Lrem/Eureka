#include "file.h"

/* 将多通道mat数据存储为txt */
void f::WriteMatToFile(mat& img, const string filename)
{
	const int img_type = img.type();

	std::ofstream fout(filename);

	if (!fout)
	{
		std::cout << "File Not Opened" << std::endl;
		return;
	}

	if (img_type == CV32FC3)
	{
		img.forEach<cv::Vec3f>([&filename, &fout](cv::Vec3f& val, const int* pos) {
			if (val[2] != 0)		// Z方向不可能为0
			{
				const string val_string = std::to_string(val[0]) + "\t" + std::to_string(val[1]) + "\t" + std::to_string(val[2]) + "\n";
				fout << val_string;
			}
			});				// 18000ms 【慢的不是存储,慢的是数据转换！！！】
	}
	else if(img_type == CV64FC3)
	{
		img.forEach<cv::Vec3d>([&filename, &fout](cv::Vec3d& val, const int* pos) {
			if (val[2] != 0)		// Z方向不可能为0
			{
				const string val_string = std::to_string(val[0]) + "\t" + std::to_string(val[1]) + "\t" + std::to_string(val[2]) + "\n";
				fout << val_string;
			}
			});
	}
}

/*存储单通道mat数据到文档内*/
void f::WriteMat(mat& img, const string filename)
{
	std::ofstream fout(filename);
	for (int m = 0; m < img.rows; m++)
	{
		for (int n = 0; n < img.cols; n++)
		{
          fout << img.at<double>(m,n) << "\t";
		}   
		fout<< "\n";
	}
	
}

/*将标定误差存储到文档中*/
void f::WriteErrorToFile(double error, const string filename)
{
	std::ofstream fout(filename);

	if (!fout)
	{
		std::cout << "File Not Opened" << std::endl;
		return;
	}
		fout << error << std::endl;
}

/* 将points数据存储为txt */
void f::Write2dPointsToFile(vector_2f points, const string filename)
{
	std::ofstream fout(filename);

	if (!fout)
	{
		std::cout << "File Not Opened" << std::endl;
		return;
	}

	for (int i = 0; i < points.size(); i++)
	{
		fout << points[i].x << "\t" << points[i].y << std::endl;
	}
}

void f::Write3dPointsToFile(vector_3f points, const string filename)
{
	std::ofstream fout(filename);

	if (!fout)
	{
		std::cout << "File Not Opened" << std::endl;
		return;
	}

	for (int i = 0; i < points.size(); i++)
	{
		fout << points[i].x << "\t" << points[i].y << "\t" << points[i].z << std::endl;
	}
}


/* 批量存储图像 */
void f::WriteMats(vector_m photos, string foldername)
{
	int num_photos = (int)photos.size();
	//std::cout << ": 拍照完成" << "W:" << photos[0].cols << "H:" << photos[0].rows << std::endl;
	for (int i = num_photos - 1; i >= 0; i--)
	{
		if (photos[i].cols != 0)
		{
			string name = "//" + std::to_string(num_photos * 10 + i) + ".bmp";
			string path = foldername + name;
			cv::imwrite(path, photos[i]);
		}
	}
}

/* 存储二值图像 */
void f::WriteMatsTo1bit(vector_m photos, string foldername)
{
	
}

/* 读取图片按文件夹 */
vector_m f::GetMats(string foldername)
{
	vector_m photos;
	vector_s photos_path;
	LOG_STRING(foldername);
	cv::glob(foldername, photos_path, false);
	for (string& img_path : photos_path)
	{
		mat temp_img = cv::imread(img_path, cv::IMREAD_UNCHANGED);
		photos.emplace_back(temp_img);	// emplace_back的性能优于push_back, IMREAD_GRAYSCALE:返回类型为8UC1
	}

	return photos;
}


/* 读取图片文件按文件名 */
vector_m f::GetMatsByFiles(string* filenames)
{
	return {};
}

/* 判断是否存在目标文件 */
bool f::FindFile(string filename)
{
	std::ifstream f(filename.c_str());
	return f.good();
}

//txt转化为pcd
//void f::CreateCloudFromTxt(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//	std::ifstream file(file_path.c_str());
//	std::string line;
//	pcl::PointXYZ point;
//	while (getline(file, line)) {
//		std::stringstream ss(line);
//		ss >> point.x;
//		ss >> point.y;
//		ss >> point.z;
//		cloud->push_back(point);
//	}
//	file.close();
//}