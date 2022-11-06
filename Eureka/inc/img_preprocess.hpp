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

    /* ����Ӧ��ֵ��ֵ��������OTSU (copy from CSDN), 
        maxval: ��ֵ�����ֵ���ͣ�Ĭ��Ϊ�ڰ�����(maxval = 255)
    */
    void ThresholdAuto(mat& src, mat& dst,int maxval = WHITEVAL);

    /* ͼ��ȥ��
        (cv:: �� + �ղ���)
        ��ȥ���ͼ������: 
    */
    void RemoveSmallRegion(mat& src, mat& dst);

};


void ipo::ThresholdAuto(mat& src, mat& dst, int maxval)
{
    int thersh = 0;

    double varValue = 0; //��䷽���м�ֵ����
    double w0 = 0; //ǰ�����ص�����ռ����
    double w1 = 0; //�������ص�����ռ����
    double u0 = 0; //ǰ��ƽ���Ҷ�
    double u1 = 0; //����ƽ���Ҷ�
    double Histogram[256] = { 0 }; //�Ҷ�ֱ��ͼ���±��ǻҶ�ֵ�����������ǻҶ�ֵ��Ӧ�����ص�����
    uchar* data = src.data;
    double totalNum = src.rows * src.cols; //��������
    //����Ҷ�ֱ��ͼ�ֲ���Histogram�����±��ǻҶ�ֵ�����������ǻҶ�ֵ��Ӧ���ص���
    for (int i = 0; i < src.rows; i++)   //Ϊ������������û�а�rows��cols���������
    {
        for (int j = 0; j < src.cols; j++)
        {
            Histogram[data[i * src.step + j]]++;
        }
    }
    for (int i = 0; i < 255; i++)
    {
        //ÿ�α���֮ǰ��ʼ��������
        w1 = 0;		u1 = 0;		w0 = 0;		u0 = 0;
        //***********����������ֵ����**************************
        for (int j = 0; j <= i; j++) //�������ָ�ֵ����
        {
            w1 += Histogram[j];  //�����������ص�����
            u1 += j * Histogram[j]; //�������������ܻҶȺ�
        }
        if (w1 == 0) //�����������ص���Ϊ0ʱ�˳�
        {
            continue;
        }
        u1 = u1 / w1; //��������ƽ���Ҷ�
        w1 = w1 / totalNum; // �����������ص�����ռ����
        //***********����������ֵ����**************************

        //***********ǰ��������ֵ����**************************
        for (int k = i + 1; k < 255; k++)
        {
            w0 += Histogram[k];  //ǰ���������ص�����
            u0 += k * Histogram[k]; //ǰ�����������ܻҶȺ�
        }
        if (w0 == 0) //ǰ���������ص���Ϊ0ʱ�˳�
        {
            break;
        }
        u0 = u0 / w0; //ǰ������ƽ���Ҷ�
        w0 = w0 / totalNum; // ǰ���������ص�����ռ����
        //***********ǰ��������ֵ����**************************

        //***********��䷽�����******************************
        double varValueI = w0 * w1 * (u1 - u0) * (u1 - u0); //��ǰ��䷽�����
        if (varValue < varValueI)
        {
            varValue = varValueI;
            thersh = i;
        }
    }

    cv::threshold(src, dst, thersh, maxval, cv::THRESH_BINARY);	// ����Ӧ��ֵ��ֵ��

}


void ipo::RemoveSmallRegion(mat& src, mat& dst)
{

    /* ���ܲ��ܻ�������ֲ����������㷨Ч�ʡ� */
    mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10)); //�����ṹԪ�ش�СΪ10*10
   
    cv::morphologyEx(src, dst, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(dst, dst, cv::MORPH_OPEN, kernel);

}
