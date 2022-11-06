#pragma once
#include <iostream>
#include <fstream>
#include <opencv.hpp>
#include <vector>
#include "global_name.h"

//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>  //���ƽ��
//#include <pcl/sample_consensus/sac_model_sphere.h> //�������
//#include <pcl/visualization/pcl_visualizer.h>  //���ƿ��ӻ�
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ�
//#include <pcl/filters/voxel_grid.h>//���ƽ�����ͷ�ļ�
//#include <pcl/filters/statistical_outlier_removal.h>//����ȥ��ͷ�ļ�
//#include <Eigen/Dense>
//#include <Eigen/Sparse>



/* ���Ժ������� */
namespace f
{
	void WriteMatToFile(mat& img, const string filename);
	void WriteErrorToFile(double error, const string filename);
	void Write2dPointsToFile(vector_2f points, const string filename);
	void Write3dPointsToFile(vector_3f points, const string filename);
	void WriteMat(mat& img, const string filename);
	void WriteMats(vector_m photos, string foldername);
	void WriteMatsTo1bit(vector_m photos, string foldername);
	int MatTo2bit(const  cv::Mat img, int  line_byte, char* data);
	bool FindFile(string filename);
	//txtת��Ϊpcd
	//void CreateCloudFromTxt(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	vector_m GetMats(string photos_folder_path);
	vector_m GetMatsByFiles(string* flienames);

	enum IMGTYPE
	{
		CV32FC3 = 21,
		CV64FC3 = 22,
	};
}