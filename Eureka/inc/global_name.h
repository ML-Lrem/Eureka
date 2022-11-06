#pragma once

// 为了方便调试而构建的函数
#define LOG_VAR(var) std::cout << var << std::endl
#define LOG_STRING(chars) std::cout << chars << std::endl
#define LOG_STRING_VAR(chars,var) std::cout << chars << ":" << var << std::endl


/* 对应头文件中的命名空间别名指定 */
// ALL
using string = std::string;
using uchar = unsigned char;
using mat = cv::Mat;
using Point2f = cv::Point2f;
using Point3f = cv::Point3f;
using vector_m = std::vector<mat>;
using vector_s = std::vector<string>;
using vector_2f = std::vector<Point2f>;
using vector_3f = std::vector<Point3f>;
using v_vector_2f = std::vector<std::vector<Point2f>>;
using v_vector_3f = std::vector<std::vector<Point3f>>;


/* 对应头文件中的常量定义 */
// ALL
constexpr float PI = 3.1415926f;		// constexpr: 常量属性
const string MEASURE_CAMERA_FOLDER = "D://Eureka//Eureka//output//measure//camera";			// 测量状态时，相机拍摄的图片保存位置

// Main
enum NumGrayPhase
{
	NUM_MASK = 2,
	NUM_GRAY = 8,			// 7 + 1 一张互补格雷码
	NUM_PHASE = 4,	
	PERIOD = 8,				// 相位周期
};

enum IndexGrayPhase
{
	INDEX_MASK = 0,
	INDEX_GRAY = 2,
	INDEX_PHASE = 10,
	INDEX_TEST = 14,
	INDEX_WHITE = 15,
};

enum MultifrePhase
{
	NUM_M_MASK = 2,
	NUM_M_PHASE = 12,        //三频四相
	M_PERIOD = 16,  
};

enum IndexMultifrePhase
{
	INDEX_M_MASK = 0,
	INDEX_M_PHASE = 2,
	INDEX_M_TEST = 14,
};

enum fMultifrePhase
{
	F1 = 70,   //三种频率
	F2 = 64,
	F3 = 59,
	N = 4,   //4步相移
};


enum ScanStatus
{
	SCAN_STOP = 0,				// 停止扫描 = 暂停+断开连接
	SCAN_START = 1,				// 开始扫描
	SCAN_PAUSE = 2,				// 暂停扫描 = 清空缓冲区，低功耗模式
};

// hardware
struct CalibrateConfig
{
	const int num_calibrate = 2;							// 标定图张数
	const float distance_board_point[2] = { 20,20 };		// 标定板特征点横纵距离，单位: mm
	const float distance_projector_point[2] = { 60,30 };		// 投影图特征点横纵距离，单位: pixel
	const int num_board_point[2] = { 11,8 };					// 标定板特征点横纵个数，width,height
	const int num_projector_point[2] = { 11,14  };				// 投影仪标定图特征点横纵个数，width,height
	const int calibrate_img_index = 14;							// 棋盘格图片(标定图)索引
	const int white_img_index = 15;								// 白图
	const string camera_calibrate_param_folder = "D://Eureka//Eureka//output//calibrate//camera_calibrate_param";	// 标定参数存放文件夹
	const string projector_calibrate_param_folder = "D://Eureka//Eureka//output//calibrate//projector_calibrate_param";	// 标定参数存放文件夹
	const string camera_photos_folder = "D://Eureka//Eureka//output//calibrate//camera_photos";						// 标定板图片存放文件夹
	const string projector_photos_folder = "D://Eureka//Eureka//output//calibrate//projector_photos";				// 标定图图片存放文件夹
	const string mask_photo = "D://Eureka//Eureka//output//measure//test//mask";
	const string gray_photo = "D://Eureka//Eureka//output//measure//test//gray";
	const string phase_photo = "D://Eureka//Eureka//output//measure//test//phase";
};

struct CheckPatternSequenceIndex
{
	int pattern_num = 1;
	int flash_index[1] = { 2 };		             // flash中24位图的索引
	int pattern_index[1] = { 0 };	             // pattern 索引（图案索引）
	int bit_depth[1] = { 1 };					 // 图像位深度
};

struct CalibratePatternSequenceIndex
{
	int pattern_num          = 2;
	int flash_index[2]   = { 0,2 };		             // flash中24位图的索引
	int pattern_index[2] = { 1,0 };	             // pattern 索引（图案索引）
	int bit_depth[2]     = { 1,1 };					 // 图像位深度
};

struct ScanPatternSequenceIndex
{
	int pattern_num = 14;
	int flash_index[14] = { 0,0,0,0,0,0,0,0,0,0,0,1,1,1 };		// flash中24位图的索引
	int pattern_index[14] = { 0,1,2,3,4,5,6,7,8,9,2,0,1,2 };	// pattern 索引（图案索引）
	int bit_depth[14] = { 1,1,1,1,1,1,1,1,1,1,8,8,8,8 };		// 图像位深度
};



// decode
constexpr bool DENOISE = false;		// 解码时：开启去噪【对运行速度影响严重】
#define DECODEIMGW = true;			// 解码时：存储中间图像


