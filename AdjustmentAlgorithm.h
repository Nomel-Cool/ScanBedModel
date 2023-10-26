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
	/// ����ʼ��
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
	/// ͨ�����ĸ����ǵ�������ݼ�������ʵ��ƽ��ķ�����
	/// </summary>
	/// <param name="H_A">A���������</param>
	/// <param name="H_B">B���������</param>
	/// <param name="H_C">C���������</param>
	/// <param name="H_D">D���������</param>
	/// <returns>���Ľǵ����ݶ�Ϊ��������true�����򷵻�false</returns>
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
	/// ͨ���������ĵ�������ݼ�������ƫת�Ƕ�
	/// </summary>
	/// <param name="bed_center_height">ͨ�����ͼ���õĴ������ĸ߶�</param>
	/// <returns>�������������Ϊ��������true�����򷵻�false</returns>
	bool CalCameraDeviationRadian(double bed_center_height)
	{
		if (bed_center_height <= 0) return false;
		m_deviation_radian = acos(bed_center_height / m_camera_height);
		return true;
	}

	/// <summary>
	/// �����ֵ�������ж�ѡȡ�߶�У����ʽ
	/// </summary>
	/// <returns>��ֵ����</returns>
	double CalKsi()
	{
		auto e = m_camera_deviation_vector.cross3(Vector3d(0, 0, -1));
		double ksi = e.normalized().dot(Vector3d(1,0,0));
		return ksi;
	}

	/// <summary>
	/// ���ݦ�ֵ�ж�ʹ�ù�ʽ���㴲���ĵ��������ƽ��ľ���
	/// </summary>
	/// <param name="ksi">�λ���</param>
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

	#pragma region ����������ú���
	/// <summary>
	/// �����������߶�
	/// </summary>
	/// <param name="height">����߶�����</param>
	/// <returns>����������߶�Ϊ��������true�����򷵻�false</returns>
	bool SetCameraPhysicalHeight(double height)
	{
		if (height <= 0) return false;
		m_camera_height = height;
		return true;
	}

	#pragma endregion

	#pragma region ���������ú���
	/// <summary>
	/// ���ô���Z�᷽�򲽽�����
	/// </summary>
	/// <param name="step_length">������������</param>
	/// <returns>��������������Ϊ�Ǹ�������true�����򷵻�false</returns>
	bool SetBedStepLength(double step_length)
	{
		if (step_length < 0) return false;
		m_bed_step = step_length;
		return true;
	}

	/// <summary>
	/// ���ô���y��̧�߾���
	/// </summary>
	/// <param name="bed_height">̧�߾�������</param>
	/// <returns>������̧�߾���Ϊ�Ǹ�������true�����򷵻�false</returns>
	bool SetBedHeight(double bed_height)
	{
		if (bed_height < 0) return false;
		m_bed_height = bed_height;
		return true;
	}
	
	/// <summary>
	/// ���ڳ�ʼ�������ô������ʹ��߶��ǰ�ȫ�ģ�������Բ������ֱ�Ӹ��´�����������
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

	#pragma region ��鴲����
	double m_bed_width;				 // ����
	double m_bed_length;			 // ����
	double m_bed_height;			 // ����
	double m_bed_step;				 // ����Z�Ჽ������
	Vector3d m_bed_current_center;	 // ����ǰ��������
	#pragma endregion

	#pragma region �������
	double m_camera_height;			 // �������߶�
	double m_deviation_radian;		 // ���ƫת����
	Vector3d m_camera_deviation_vector; // �����ʵƽ�淨����
	#pragma endregion

};

#endif

