#pragma once
#ifndef SPHERE_COORDINATE_SYSTEM_H
#define SPHERE_COORDINATE_SYSTEM_H

#include <map>

#include "eigen/Eigen"

constexpr double M_PI = 3.14159265358979323846;
using namespace Eigen;
class SphereCoordinate
{
public:
	/// <summary>
	/// ������ϵ��ʼ����Ĭ�ϵ�λ��(0,0,0)Ϊԭ��
	/// </summary>
	SphereCoordinate();

	#pragma region ����������ϵ��������
		/// <summary>
		/// ���æսǻ���ֵ
		/// </summary>
		/// <param name="inPhiAngle">����սǽǶ�ֵ</param>
		/// <returns>������[0��,180��]�򷵻�true�����򷵻�false</returns>
		bool SetPhi(double inPhiAngle);

		/// <summary>
		/// ���æȽǻ���ֵ
		/// </summary>
		/// <param name="inThetaAngle">����ȽǽǶ�ֵ</param>
		/// <returns>������[0��,360��]�򷵻�true�����򷵻�false</returns>
		bool SetTheta(double inThetaAngle);

		/// <summary>
		/// ���æ�ֵ
		/// </summary>
		/// <param name="inRho">������뾶</param>
		/// <returns>������İ뾶����0����true�����򷵻�false</returns>
		bool SetRho(double inRho);
	#pragma endregion

	#pragma region ��ȡ���ٱ���
		/// <summary>
		/// ��ȡ�Ѽ������sin(phi)ֵ
		/// </summary>
		/// <param name="phi">��Ҫ�����phiֵ</param>
		/// <returns>sin(phi)</returns>
		double GetSinPhi(double phi);

		/// <summary>
		/// ��ȡ�Ѽ������cos(phi)ֵ
		/// </summary>
		/// <param name="phi">��Ҫ�����phiֵ</param>
		/// <returns>cos(phi)</returns>
		double GetCosPhi(double phi);

		/// <summary>
		/// ��ȡ�Ѽ������sin(theta)ֵ
		/// </summary>
		/// <param name="theta">��Ҫ�����thetaֵ</param>
		/// <returns>sin(theta)</returns>
		double GetSinTheta(double theta);

		/// <summary>
		/// ��ȡ�Ѽ������cos(theta)ֵ
		/// </summary>
		/// <param name="theta">��Ҫ�����thetaֵ</param>
		/// <returns>cos(theta)</returns>
		double GetCosTheta(double theta);
	#pragma endregion

	/// <summary>
	/// ������ת�ѿ�������
	/// </summary>
	/// <returns>���ص�ǰ������</returns>
	Vector3d ConvertSphere2RectangleSystem();

private:

	#pragma region ������ϵ���˽�г�Ա����
		double m_phi;		  // �սǻ���
		double m_rho;		  // ����뾶
		double m_theta;		  // �Ƚǻ���
		Vector3d m_origin;	  // ԭ��ѿ�������
	#pragma endregion

	#pragma region ���ټ������˽�б���
		std::map<double, double> m_sin_phi;   // �����Ѽ������sin(m_phi)ֵ
		std::map<double, double> m_cos_phi;   // �����Ѽ������cos(m_phi)ֵ
		std::map<double, double> m_sin_theta; // �����Ѽ������sin(m_theta)ֵ 
		std::map<double, double> m_cos_theta; // �����Ѽ������cos(m_theta)ֵ 
	#pragma endregion

};

#endif // !SPHERE_COORDINATE_SYSTEM_H
