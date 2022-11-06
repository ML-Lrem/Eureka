#include "imgcode.h"

vector_m imgcode::GreatMaskCode()
{
	vector_m mask_code;

	mask_code.emplace_back(mat::zeros(height_, width_, CV_8UC1));			// ���ɺ�ɫ
	mask_code.emplace_back(mat::ones(height_, width_, CV_8UC1) * 255);		// ���ɰ�ɫ

	return mask_code;
}


vector_m imgcode::GreatGrayCode()
{
	vector_m gray_code;

	int change_width;
	bool odd_even;
	int start_col = 0;
	int end_col = 0;
	int count_stripe;
	int period;

	int total_split = (int)pow(2, num_gray_ - 1) * num_phase_;			// �����ǻ�������������
	change_width = width_ + (total_split - width_ % total_split);		// ��չ��� ��֤width�ܱ�(2^num_gray_*num_phase)����,��Ϊ�������Ʋ���

	LOG_STRING_VAR("total_split", total_split);
	LOG_STRING_VAR("change_width", change_width);

	for (int i = 0; i < num_gray_; i++)
	{
		mat temp_img_code = mat::zeros(height_, change_width, CV_8UC1);

		count_stripe = (int)pow(2, i) + 1;							// ��ǰ��������
		period = change_width / (count_stripe - 1);			    // ��ǰ��������

		for (int j = 0; j < count_stripe; j++)
		{
			odd_even = j % 2;									// ������ż��
			if (j == 0)											// ��һ�顢���һ��(��0��ʼ�����һ�����=��������-1)
			{
				start_col = 0;
				end_col = start_col + period / 2;
			}
			else if (j == count_stripe - 1)
			{
				start_col = end_col;
				end_col = change_width;
			}
			else
			{
				start_col = end_col;
				end_col = start_col + period;
			}
			temp_img_code.colRange(start_col, end_col) = temp_img_code.colRange(start_col, end_col) + 255 * odd_even;
		}

		mat img_code = mat::zeros(height_, width_, CV_8UC1);
		img_code = img_code + temp_img_code.colRange(0, width_);

		gray_code.emplace_back(img_code);			// mask.code.size()
	}

	return gray_code;
}


vector_m imgcode::GreatPhaseCode()
{
	vector_m phase_code;

	int change_width;
	int count_stripe;
	int period;
	int start_col = 0;
	int end_col = 0;

	int num_real_gray = num_gray_ - 1;

	int total_split = (int)pow(2, num_real_gray) * num_phase_;
	change_width = width_ + (total_split - width_ % total_split);		// ��չ��� ��֤width�ܱ�(2^num_gray_*num_phase)����

	count_stripe = (int)pow(2, num_real_gray);									// ��������������							
	period = change_width / count_stripe;								// ��������������


	// �������������Ƿ�Ϲ�
	bool is_legal_period = period % num_phase_ == 0;
	if (!is_legal_period) { LOG_STRING("���ڴ���"); return mat{ 0 }; }

	int col_val;
	float omega_t, phi;
	for (int i = 0; i < num_phase_; i++)
	{
		mat temp_stripe_img = mat::zeros(height_, period, CV_8UC1);			// ���ɵ����� ��������
		for (int t = 0; t < period; t++)
		{
			omega_t = 2 * PI / period * (t + 1);
			phi = 2 * PI / num_phase_;									// ÿ���ƶ�������λ
			col_val = round((cos(omega_t + phi * i) + 1) * 255 / 2);			// ��[-1,1] ��չ��[0,255]
			temp_stripe_img.colRange(t, t + 1) = temp_stripe_img.colRange(t, t + 1) + col_val;
		}

		mat temp_img_code = mat::zeros(height_, change_width, CV_8UC1);
		for (int j = 0; j < count_stripe; j++)
		{
			if (j == 0)									// ��һ������
			{
				start_col = 0;
				end_col = period;
			}
			else
			{
				start_col = end_col;
				end_col = start_col + period;
			}
			temp_img_code.colRange(start_col, end_col) = temp_img_code.colRange(start_col, end_col) + temp_stripe_img;
		}

		mat img_code = mat::zeros(height_, width_, CV_8UC1);
		img_code = img_code + temp_img_code.colRange(0, width_);

		phase_code.emplace_back(img_code);
	}

	return phase_code;
}


mat imgcode::GreatChecker(int distance)
{
	mat img(height_, width_, CV_8UC1);

	img.forEach<uchar>([&distance](uchar& val, const int *pos) {
		int temp_width = (int)(pos[0] / distance);
		int temp_height = (int)(pos[1] / distance);
		if (temp_width % 2 == temp_height % 2) { val = 255; } 		// ��ż����ͬ��Ϊ��
		else { val = 0; }

		});
	return img;
}

int imgcode::GetPeriod()
{
	int change_width;
	int count_stripe;

	int num_real_gray = num_gray_ - 1;

	int total_split = (int)pow(2, num_real_gray) * num_phase_;
	change_width = width_ + (total_split - width_ % total_split);		// ��չ��� ��֤width�ܱ�(2^num_gray_*num_phase)����

	count_stripe = (int)pow(2, num_real_gray);									// ��������������							
	period_ = change_width / count_stripe;								// ��������������}

	return period_;
}

void imgcode::SetNumGray(int num_gray) { num_gray_ = num_gray; }
void imgcode::SetNumPhase(int num_phase) { num_phase_ = num_phase; }
