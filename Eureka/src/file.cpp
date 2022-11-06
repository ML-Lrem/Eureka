#include "file.h"

/* ����ͨ��mat���ݴ洢Ϊtxt */
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
			if (val[2] != 0)		// Z���򲻿���Ϊ0
			{
				const string val_string = std::to_string(val[0]) + "\t" + std::to_string(val[1]) + "\t" + std::to_string(val[2]) + "\n";
				fout << val_string;
			}
			});				// 18000ms �����Ĳ��Ǵ洢,����������ת����������
	}
	else if(img_type == CV64FC3)
	{
		img.forEach<cv::Vec3d>([&filename, &fout](cv::Vec3d& val, const int* pos) {
			if (val[2] != 0)		// Z���򲻿���Ϊ0
			{
				const string val_string = std::to_string(val[0]) + "\t" + std::to_string(val[1]) + "\t" + std::to_string(val[2]) + "\n";
				fout << val_string;
			}
			});
	}
}

/*�洢��ͨ��mat���ݵ��ĵ���*/
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

/*���궨���洢���ĵ���*/
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

/* ��points���ݴ洢Ϊtxt */
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


/* �����洢ͼ�� */
void f::WriteMats(vector_m photos, string foldername)
{
	int num_photos = (int)photos.size();
	//std::cout << ": �������" << "W:" << photos[0].cols << "H:" << photos[0].rows << std::endl;
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

/* �洢��ֵͼ�� */
void f::WriteMatsTo1bit(vector_m photos, string foldername)
{
	
}

/* ��ȡͼƬ���ļ��� */
vector_m f::GetMats(string foldername)
{
	vector_m photos;
	vector_s photos_path;
	LOG_STRING(foldername);
	cv::glob(foldername, photos_path, false);
	for (string& img_path : photos_path)
	{
		mat temp_img = cv::imread(img_path, cv::IMREAD_UNCHANGED);
		photos.emplace_back(temp_img);	// emplace_back����������push_back, IMREAD_GRAYSCALE:��������Ϊ8UC1
	}

	return photos;
}


/* ��ȡͼƬ�ļ����ļ��� */
vector_m f::GetMatsByFiles(string* filenames)
{
	return {};
}

/* �ж��Ƿ����Ŀ���ļ� */
bool f::FindFile(string filename)
{
	std::ifstream f(filename.c_str());
	return f.good();
}

//txtת��Ϊpcd
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