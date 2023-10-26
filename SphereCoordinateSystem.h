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
	/// 球坐标系初始化，默认单位球，(0,0,0)为原点
	/// </summary>
	SphereCoordinate();

	#pragma region 设置球坐标系参数方法
		/// <summary>
		/// 设置φ角弧度值
		/// </summary>
		/// <param name="inPhiAngle">输入φ角角度值</param>
		/// <returns>若输入[0°,180°]则返回true，否则返回false</returns>
		bool SetPhi(double inPhiAngle);

		/// <summary>
		/// 设置θ角弧度值
		/// </summary>
		/// <param name="inThetaAngle">输入θ角角度值</param>
		/// <returns>若输入[0°,360°]则返回true，否则返回false</returns>
		bool SetTheta(double inThetaAngle);

		/// <summary>
		/// 设置ρ值
		/// </summary>
		/// <param name="inRho">输入球半径</param>
		/// <returns>若输入的半径大于0返回true，否则返回false</returns>
		bool SetRho(double inRho);
	#pragma endregion

	#pragma region 获取加速表方法
		/// <summary>
		/// 获取已计算过的sin(phi)值
		/// </summary>
		/// <param name="phi">需要计算的phi值</param>
		/// <returns>sin(phi)</returns>
		double GetSinPhi(double phi);

		/// <summary>
		/// 获取已计算过的cos(phi)值
		/// </summary>
		/// <param name="phi">需要计算的phi值</param>
		/// <returns>cos(phi)</returns>
		double GetCosPhi(double phi);

		/// <summary>
		/// 获取已计算过的sin(theta)值
		/// </summary>
		/// <param name="theta">需要计算的theta值</param>
		/// <returns>sin(theta)</returns>
		double GetSinTheta(double theta);

		/// <summary>
		/// 获取已计算过的cos(theta)值
		/// </summary>
		/// <param name="theta">需要计算的theta值</param>
		/// <returns>cos(theta)</returns>
		double GetCosTheta(double theta);
	#pragma endregion

	/// <summary>
	/// 球坐标转笛卡尔坐标
	/// </summary>
	/// <returns>返回当前球坐标</returns>
	Vector3d ConvertSphere2RectangleSystem();

private:

	#pragma region 球坐标系相关私有成员变量
		double m_phi;		  // φ角弧度
		double m_rho;		  // ρ球半径
		double m_theta;		  // θ角弧度
		Vector3d m_origin;	  // 原点笛卡尔坐标
	#pragma endregion

	#pragma region 加速计算相关私有变量
		std::map<double, double> m_sin_phi;   // 保存已计算过的sin(m_phi)值
		std::map<double, double> m_cos_phi;   // 保存已计算过的cos(m_phi)值
		std::map<double, double> m_sin_theta; // 保存已计算过的sin(m_theta)值 
		std::map<double, double> m_cos_theta; // 保存已计算过的cos(m_theta)值 
	#pragma endregion

};

#endif // !SPHERE_COORDINATE_SYSTEM_H
