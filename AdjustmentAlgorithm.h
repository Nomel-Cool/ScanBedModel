#pragma once
#ifndef ADJUSTMENT_ALGORITHM_H
#define ADJUSTMENT_ALGORITHM_H

#include "eigen/Eigen"

#include "SphereCoordinateSystem.h"
using namespace Eigen;

class Adjustment
{
public:
	/// <summary>
	/// 床初始化
	/// </summary>
	Adjustment()
	{
		m_bed_width = 0.0;
		m_bed_length = 0.0;
		m_bed_height = 0.0;
		m_bed_step = 0.0;
		m_camera_height = 0.0;
		m_deviation_radian = 0.0;
		m_camera_deviation_vector << 0.0, 0.0, 0.0;
		m_bed_current_center << 0.0, 0.0, 0.0;
	}

	/// <summary>
	/// 通过床四个顶角的深度数据计算出相机实际平面的法向量
	/// </summary>
	/// <param name="H_A">A角深度数据</param>
	/// <param name="H_B">B角深度数据</param>
	/// <param name="H_C">C角深度数据</param>
	/// <param name="H_D">D角深度数据</param>
	/// <returns>若四角的数据都为正数返回true，否则返回false</returns>
	bool GalCameraDeviationNormalVector(double H_A, double H_B, double H_C, double H_D)
	{
		if (H_A <= 0 || H_B <= 0 || H_C <= 0 || H_D <= 0)return false;
		Vector3d A(m_bed_length * -0.5, m_bed_width * 0.5, H_A);
		Vector3d B(m_bed_length * -0.5, m_bed_width * -0.5, H_B);
		Vector3d C(m_bed_length * 0.5, m_bed_width * 0.5, H_C);
		Vector3d D(m_bed_length * 0.5, m_bed_width * -0.5, H_D);
		Vector3d AD = D - A;
		Vector3d BC = C - B;
		m_camera_deviation_vector = AD.cross3(BC);
		return true;
	}

	/// <summary>
	/// 通过床正中心的深度数据计算出相机偏转角度
	/// </summary>
	/// <param name="bed_center_height">通过深度图像获得的床正中心高度</param>
	/// <returns>若给出深度数据为正数返回true，否则返回false</returns>
	bool CalCameraDeviationRadian(double bed_center_height)
	{
		if (bed_center_height <= 0) return false;
		m_deviation_radian = acos(bed_center_height / m_camera_height);
		return true;
	}

	/// <summary>
	/// 计算ξ值，用于判断选取高度校正公式
	/// </summary>
	/// <returns>ξ值弧度</returns>
	double CalKsi()
	{
		auto e = m_camera_deviation_vector.cross3(Vector3d(0, 0, -1));
		double ksi = e.normalized().dot(Vector3d(1,0,0));
		return ksi;
	}

	/// <summary>
	/// 根据ξ值判断使用公式计算床中心到相机理想平面的距离
	/// </summary>
	/// <param name="ksi">ξ弧度</param>
	/// <returns></returns>
	double CalRealHeight(double ksi, double data_height)
	{
		if (ksi >= M_PI * -0.5 && ksi <= M_PI * 0.5)
		{
			return data_height / cos(m_deviation_radian) - m_bed_step * tan(m_deviation_radian) * sin(ksi);
		}
		else
		{
			return data_height / cos(m_deviation_radian) + m_bed_step * tan(m_deviation_radian) * sin(ksi);
		}
	}

	#pragma region 相机参数设置函数
	/// <summary>
	/// 设置相机物理高度
	/// </summary>
	/// <param name="height">物理高度数据</param>
	/// <returns>若给出物理高度为正数返回true，否则返回false</returns>
	bool SetCameraPhysicalHeight(double height)
	{
		if (height <= 0) return false;
		m_camera_height = height;
		return true;
	}

	#pragma endregion

	#pragma region 床参数设置函数
	/// <summary>
	/// 设置床向Z轴方向步进距离
	/// </summary>
	/// <param name="step_length">步进距离数据</param>
	/// <returns>若给出步进距离为非负数返回true，否则返回false</returns>
	bool SetBedStepLength(double step_length)
	{
		if (step_length < 0) return false;
		m_bed_step = step_length;
		return true;
	}

	/// <summary>
	/// 设置床向y轴抬高距离
	/// </summary>
	/// <param name="bed_height">抬高距离数据</param>
	/// <returns>若给出抬高距离为非负数返回true，否则返回false</returns>
	bool SetBedHeight(double bed_height)
	{
		if (bed_height < 0) return false;
		m_bed_height = bed_height;
		return true;
	}
	
	/// <summary>
	/// 由于初始化和设置床步进和床高都是安全的，这里可以不作检测直接更新床正中心坐标
	/// </summary>
	/// <returns></returns>
	void UpdateBedCenter()
	{
		m_bed_current_center(0) = m_bed_step;
		m_bed_current_center(1) = 0.0;
		m_bed_current_center(2) = m_bed_height;
	}

#pragma endregion

private:

	#pragma region 检查床参数
	double m_bed_width;				 // 床宽
	double m_bed_length;			 // 床长
	double m_bed_height;			 // 床高
	double m_bed_step;				 // 床向Z轴步进距离
	Vector3d m_bed_current_center;	 // 床当前中心坐标
	#pragma endregion

	#pragma region 相机参数
	double m_camera_height;			 // 相机物理高度
	double m_deviation_radian;		 // 相机偏转弧度
	Vector3d m_camera_deviation_vector; // 相机真实平面法向量
	#pragma endregion

};

#endif

