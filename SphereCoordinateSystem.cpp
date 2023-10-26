#include "SphereCoordinateSystem.h"

SphereCoordinate::SphereCoordinate()
{
	m_rho = 1.0;
	m_phi = 0.0;
	m_theta = 0.0;
	m_origin << 0.0, 0.0, 0.0;
}

bool SphereCoordinate::SetPhi(double inPhiAngle)
{
	if (inPhiAngle >= 0.0 && inPhiAngle <= 180.0)
	{
		m_phi = inPhiAngle * M_PI / 180.0;
		return true;
	}
	return false;
}

bool SphereCoordinate::SetTheta(double inThetaAngle)
{
	if (inThetaAngle >= 0.0 && inThetaAngle <= 360.0)
	{
		m_theta = inThetaAngle * M_PI / 180.0;
		return true;
	}
	return false;
}

bool SphereCoordinate::SetRho(double inRho)
{
	if (inRho > 0.0)
	{
		m_rho = inRho;
		return true;
	}
	return false;
}

double SphereCoordinate::GetSinPhi(double phi)
{
	if (m_sin_phi.find(phi) != m_sin_phi.end())
		return m_sin_phi[phi];
	else
	{
		double value = sin(phi);
		m_sin_phi[phi] = value;
		return value;
	}
}

double SphereCoordinate::GetCosPhi(double phi)
{
	if (m_cos_phi.find(phi) != m_cos_phi.end())
		return m_cos_phi[phi];
	else
	{
		double value = cos(phi);
		m_cos_phi[phi] = value;
		return value;
	}
}

double SphereCoordinate::GetSinTheta(double theta)
{
	if (m_sin_theta.find(theta) != m_sin_theta.end())
		return m_sin_theta[theta];
	else
	{
		double value = sin(theta);
		m_sin_theta[theta] = value;
		return value;
	}
}

double SphereCoordinate::GetCosTheta(double theta)
{
	if (m_cos_theta.find(theta) != m_cos_theta.end())
		return m_cos_theta[theta];
	else
	{
		double value = cos(theta);
		m_cos_theta[theta] = value;
		return value;
	}
}

Vector3d SphereCoordinate::ConvertSphere2RectangleSystem()
{
	double x = m_phi * GetSinPhi(m_phi) * GetCosTheta(m_theta);
	double y = m_phi * GetSinPhi(m_phi) * GetSinTheta(m_theta);
	double z = m_phi * GetCosPhi(m_phi);
	return Vector3d(x, y, z);
}