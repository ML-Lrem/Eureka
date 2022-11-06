#include "imgcode.h"

vector_m imgcode::GreatMaskCode()
{
	vector_m mask_code;

	mask_code.emplace_back(mat::zeros(height_, width_, CV_8UC1));			// 生成黑色
	mask_code.emplace_back(mat::ones(height_, width_, CV_8UC1) * 255);		// 生成白色

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

	int total_split = (int)pow(2, num_gray_ - 1) * num_phase_;			// 不考虑互补格雷码的情况
	change_width = width_ + (total_split - width_ % total_split);		// 拓展宽度 保证width能被(2^num_gray_*num_phase)整除,因为还有相移部分

	LOG_STRING_VAR("total_split", total_split);
	LOG_STRING_VAR("change_width", change_width);

	for (int i = 0; i < num_gray_; i++)
	{
		mat temp_img_code = mat::zeros(height_, change_width, CV_8UC1);

		count_stripe = (int)pow(2, i) + 1;							// 当前条纹数量
		period = change_width / (count_stripe - 1);			    // 当前条纹周期

		for (int j = 0; j < count_stripe; j++)
		{
			odd_even = j % 2;									// 计算奇偶列
			if (j == 0)											// 第一组、最后一组(从0开始，最后一组序号=条纹总数-1)
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
	change_width = width_ + (total_split - width_ % total_split);		// 拓展宽度 保证width能被(2^num_gray_*num_phase)整除

	count_stripe = (int)pow(2, num_real_gray);									// 相移码条纹数量							
	period = change_width / count_stripe;								// 相移码条纹周期


	// 检验相移周期是否合规
	bool is_legal_period = period % num_phase_ == 0;
	if (!is_legal_period) { LOG_STRING("周期错误"); return mat{ 0 }; }

	int col_val;
	float omega_t, phi;
	for (int i = 0; i < num_phase_; i++)
	{
		mat temp_stripe_img = mat::zeros(height_, period, CV_8UC1);			// 生成单条纹 加上相移
		for (int t = 0; t < period; t++)
		{
			omega_t = 2 * PI / period * (t + 1);
			phi = 2 * PI / num_phase_;									// 每次移动多少相位
			col_val = round((cos(omega_t + phi * i) + 1) * 255 / 2);			// 从[-1,1] 拓展到[0,255]
			temp_stripe_img.colRange(t, t + 1) = temp_stripe_img.colRange(t, t + 1) + col_val;
		}

		mat temp_img_code = mat::zeros(height_, change_width, CV_8UC1);
		for (int j = 0; j < count_stripe; j++)
		{
			if (j == 0)									// 第一个条纹
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
		if (temp_width % 2 == temp_height % 2) { val = 255; } 		// 奇偶性相同则为零
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
	change_width = width_ + (total_split - width_ % total_split);		// 拓展宽度 保证width能被(2^num_gray_*num_phase)整除

	count_stripe = (int)pow(2, num_real_gray);									// 相移码条纹数量							
	period_ = change_width / count_stripe;								// 相移码条纹周期}

	return period_;
}

void imgcode::SetNumGray(int num_gray) { num_gray_ = num_gray; }
void imgcode::SetNumPhase(int num_phase) { num_phase_ = num_phase; }
