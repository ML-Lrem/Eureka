#pragma once

// Ϊ�˷�����Զ������ĺ���
#define LOG_VAR(var) std::cout << var << std::endl
#define LOG_STRING(chars) std::cout << chars << std::endl
#define LOG_STRING_VAR(chars,var) std::cout << chars << ":" << var << std::endl


/* ��Ӧͷ�ļ��е������ռ����ָ�� */
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


/* ��Ӧͷ�ļ��еĳ������� */
// ALL
constexpr float PI = 3.1415926f;		// constexpr: ��������
const string MEASURE_CAMERA_FOLDER = "D://Eureka//Eureka//output//measure//camera";			// ����״̬ʱ����������ͼƬ����λ��

// Main
enum NumGrayPhase
{
	NUM_MASK = 2,
	NUM_GRAY = 8,			// 7 + 1 һ�Ż���������
	NUM_PHASE = 4,	
	PERIOD = 8,				// ��λ����
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
	NUM_M_PHASE = 12,        //��Ƶ����
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
	F1 = 70,   //����Ƶ��
	F2 = 64,
	F3 = 59,
	N = 4,   //4������
};


enum ScanStatus
{
	SCAN_STOP = 0,				// ֹͣɨ�� = ��ͣ+�Ͽ�����
	SCAN_START = 1,				// ��ʼɨ��
	SCAN_PAUSE = 2,				// ��ͣɨ�� = ��ջ��������͹���ģʽ
};

// hardware
struct CalibrateConfig
{
	const int num_calibrate = 2;							// �궨ͼ����
	const float distance_board_point[2] = { 20,20 };		// �궨����������ݾ��룬��λ: mm
	const float distance_projector_point[2] = { 60,30 };		// ͶӰͼ��������ݾ��룬��λ: pixel
	const int num_board_point[2] = { 11,8 };					// �궨����������ݸ�����width,height
	const int num_projector_point[2] = { 11,14  };				// ͶӰ�Ǳ궨ͼ��������ݸ�����width,height
	const int calibrate_img_index = 14;							// ���̸�ͼƬ(�궨ͼ)����
	const int white_img_index = 15;								// ��ͼ
	const string camera_calibrate_param_folder = "D://Eureka//Eureka//output//calibrate//camera_calibrate_param";	// �궨��������ļ���
	const string projector_calibrate_param_folder = "D://Eureka//Eureka//output//calibrate//projector_calibrate_param";	// �궨��������ļ���
	const string camera_photos_folder = "D://Eureka//Eureka//output//calibrate//camera_photos";						// �궨��ͼƬ����ļ���
	const string projector_photos_folder = "D://Eureka//Eureka//output//calibrate//projector_photos";				// �궨ͼͼƬ����ļ���
	const string mask_photo = "D://Eureka//Eureka//output//measure//test//mask";
	const string gray_photo = "D://Eureka//Eureka//output//measure//test//gray";
	const string phase_photo = "D://Eureka//Eureka//output//measure//test//phase";
};

struct CheckPatternSequenceIndex
{
	int pattern_num = 1;
	int flash_index[1] = { 2 };		             // flash��24λͼ������
	int pattern_index[1] = { 0 };	             // pattern ������ͼ��������
	int bit_depth[1] = { 1 };					 // ͼ��λ���
};

struct CalibratePatternSequenceIndex
{
	int pattern_num          = 2;
	int flash_index[2]   = { 0,2 };		             // flash��24λͼ������
	int pattern_index[2] = { 1,0 };	             // pattern ������ͼ��������
	int bit_depth[2]     = { 1,1 };					 // ͼ��λ���
};

struct ScanPatternSequenceIndex
{
	int pattern_num = 14;
	int flash_index[14] = { 0,0,0,0,0,0,0,0,0,0,0,1,1,1 };		// flash��24λͼ������
	int pattern_index[14] = { 0,1,2,3,4,5,6,7,8,9,2,0,1,2 };	// pattern ������ͼ��������
	int bit_depth[14] = { 1,1,1,1,1,1,1,1,1,1,8,8,8,8 };		// ͼ��λ���
};



// decode
constexpr bool DENOISE = false;		// ����ʱ������ȥ�롾�������ٶ�Ӱ�����ء�
#define DECODEIMGW = true;			// ����ʱ���洢�м�ͼ��


