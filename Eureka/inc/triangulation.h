#pragma once
#include <iostream>
#include <opencv.hpp>
#include <vector>
#include "global_name.h"

/* ���ǲ��������ڶ�Ԫ���� */
mat Triangulation(scanner& scanner_user, mat& photos_decoded, mat& mask);
mat udTriangulation(scanner& scanner_user, mat& photos_decoded, mat& mask);