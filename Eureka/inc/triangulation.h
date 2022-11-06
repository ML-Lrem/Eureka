#pragma once
#include <iostream>
#include <opencv.hpp>
#include <vector>
#include "global_name.h"

/* 三角测量：基于多元方程 */
mat Triangulation(scanner& scanner_user, mat& photos_decoded, mat& mask);
mat udTriangulation(scanner& scanner_user, mat& photos_decoded, mat& mask);