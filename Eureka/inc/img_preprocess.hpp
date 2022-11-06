#pragma once
#include <iostream>
#include <opencv.hpp>
#include <vector>
#include "global_name.h"

namespace ipo
{
    enum thresholdMaskval {
        MASKVAL = 1,
        WHITEVAL = 255,
    };

    /* 自适应阈值二值化方法：OTSU (copy from CSDN), 
        maxval: 二值化最大值类型，默认为黑白类型(maxval = 255)
    */
    void ThresholdAuto(mat& src, mat& dst,int maxval = WHITEVAL);

    /* 图像去噪
        (cv:: 开 + 闭操作)
        待去噪的图像类型: 
    */
    void RemoveSmallRegion(mat& src, mat& dst);

};


void ipo::ThresholdAuto(mat& src, mat& dst, int maxval)
{
    int thersh = 0;

    double varValue = 0; //类间方差中间值保存
    double w0 = 0; //前景像素点数所占比例
    double w1 = 0; //背景像素点数所占比例
    double u0 = 0; //前景平均灰度
    double u1 = 0; //背景平均灰度
    double Histogram[256] = { 0 }; //灰度直方图，下标是灰度值，保存内容是灰度值对应的像素点总数
    uchar* data = src.data;
    double totalNum = src.rows * src.cols; //像素总数
    //计算灰度直方图分布，Histogram数组下标是灰度值，保存内容是灰度值对应像素点数
    for (int i = 0; i < src.rows; i++)   //为表述清晰，并没有把rows和cols单独提出来
    {
        for (int j = 0; j < src.cols; j++)
        {
            Histogram[data[i * src.step + j]]++;
        }
    }
    for (int i = 0; i < 255; i++)
    {
        //每次遍历之前初始化各变量
        w1 = 0;		u1 = 0;		w0 = 0;		u0 = 0;
        //***********背景各分量值计算**************************
        for (int j = 0; j <= i; j++) //背景部分各值计算
        {
            w1 += Histogram[j];  //背景部分像素点总数
            u1 += j * Histogram[j]; //背景部分像素总灰度和
        }
        if (w1 == 0) //背景部分像素点数为0时退出
        {
            continue;
        }
        u1 = u1 / w1; //背景像素平均灰度
        w1 = w1 / totalNum; // 背景部分像素点数所占比例
        //***********背景各分量值计算**************************

        //***********前景各分量值计算**************************
        for (int k = i + 1; k < 255; k++)
        {
            w0 += Histogram[k];  //前景部分像素点总数
            u0 += k * Histogram[k]; //前景部分像素总灰度和
        }
        if (w0 == 0) //前景部分像素点数为0时退出
        {
            break;
        }
        u0 = u0 / w0; //前景像素平均灰度
        w0 = w0 / totalNum; // 前景部分像素点数所占比例
        //***********前景各分量值计算**************************

        //***********类间方差计算******************************
        double varValueI = w0 * w1 * (u1 - u0) * (u1 - u0); //当前类间方差计算
        if (varValue < varValueI)
        {
            varValue = varValueI;
            thersh = i;
        }
    }

    cv::threshold(src, dst, thersh, maxval, cv::THRESH_BINARY);	// 自适应阈值二值化

}


void ipo::RemoveSmallRegion(mat& src, mat& dst)
{

    /* 【能不能混合这两种操作，提升算法效率】 */
    mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10)); //创建结构元素大小为10*10
   
    cv::morphologyEx(src, dst, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(dst, dst, cv::MORPH_OPEN, kernel);

}
