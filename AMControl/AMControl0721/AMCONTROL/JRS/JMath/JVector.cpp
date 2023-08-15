#include "TcPch.h"
#pragma hdrstop

#include "../include/JRobotSystemDefine.h"

 #include "JMath.h"
//#include "JVector.h"

bool jm_Vec_IsZero(const JVector& v1, double epsilon)
{
	return ((FABS(v1[0]) < epsilon) &&
		(FABS(v1[1]) < epsilon) &&
		(FABS(v1[2]) < epsilon));
}

double jm_Vec_distance(const JVector& v1, const JVector& v2)
{
	return jm_Vec_magnitude(v1 - v2);
}

double jm_Vec_dotProduct(const JVector& v1, const JVector& v2)
{
	return (v1[0] * v2[0] +
		v1[1] * v2[1] +
		v1[2] * v2[2]);
}

JVector jm_Vec_crossProduct(const JVector& v1, const JVector& v2)
{
	JVector result(v1[1] * v2[2] - v1[2] * v2[1],
		v1[2] * v2[0] - v1[0] * v2[2],
		v1[0] * v2[1] - v1[1] * v2[0]);
	return result;
}

JVector jm_Vec_normalize(const JVector& v1)
{
	double m = jm_Vec_magnitude(v1);
	if (m == 0) return v1;
	return v1 / m;
}

double jm_Vec_magnitude(const JVector& v1)
{
	return v1.magnitude();
}

double jm_Vec_norm(const JVector& v1)
{
	return v1.norm();
}

double jm_Vec_norm_1(const JVector& v1)
{
	return v1.norm_1();
}

double jm_Vec_norm_infinite(const JVector& v1)
{
	return v1.norm_infinite();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// JVector Class Member Function
JVector::JVector(void)
{
	Set(0.0, 0.0, 0.0);
}

JVector::JVector(double x, double y, double z)
{
	Set(x, y, z);
}

JVector::JVector(const JVector& Cpy)
{
	*this = Cpy;
}

JVector::~JVector(void)
{

}

void JVector::Set(double x, double y, double z)
{
	m_p[0] = x;
	m_p[1] = y;
	m_p[2] = z;
}

bool JVector::IsZero(double epsilon) const
{
	return jm_Vec_IsZero(*this, epsilon);
}

double JVector::distance(const JVector& v1) const
{
	return (*this - v1).magnitude();
}

double JVector::dotProduct(const JVector& v1) const
{
	return jm_Vec_dotProduct(*this, v1);
}

JVector JVector::crossProduct(const JVector& v1) const
{
	return jm_Vec_crossProduct(*this, v1);
}

void JVector::normalize(void)
{
	(*this) = jm_Vec_normalize(*this);
}

double JVector::magnitude(void) const
{
	return SQRT(m_p[0] * m_p[0] +
		m_p[1] * m_p[1] +
		m_p[2] * m_p[2]);
}

double JVector::getdistance(void)
{
	return magnitude();
}

double JVector::norm(void) const
{
	return this->magnitude();
}

double JVector::norm_1(void) const
{
	return (FABS(m_p[0]) + FABS(m_p[1]) + FABS(m_p[2]));
}

double JVector::norm_infinite(void) const
{
	return m_p[getLongestAxisComponent()];
}

int JVector::getLongestAxisComponent(void) const
{
	if (FABS(m_p[0]) > FABS(m_p[1]) && FABS(m_p[0]) > FABS(m_p[2]))
		return 0;
	if (FABS(m_p[1]) > FABS(m_p[0]) && FABS(m_p[1]) > FABS(m_p[2]))
		return 1;
	return 2;
}

int JVector::getSecondLongestAxisComponent(void) const
{
	int nLongestAxis = getLongestAxisComponent();
	int result;
	switch (nLongestAxis) {
	case 0:
		if (FABS(m_p[1]) > FABS(m_p[2]))
			result = 1;
		else
			result = 2;
		break;
	case 1:
		if (FABS(m_p[0]) > FABS(m_p[2]))
			result = 0;
		else
			result = 2;
		break;
	case 2:
		if (FABS(m_p[0]) > FABS(m_p[1]))
			result = 0;
		else
			result = 1;
		break;
	}

	return result;
}

int JVector::getShortestAxisComponent(void) const
{
	if (FABS(m_p[0]) < FABS(m_p[1]) && FABS(m_p[0]) < FABS(m_p[2]))
		return 0;
	if (FABS(m_p[1]) < FABS(m_p[0]) && FABS(m_p[1]) < FABS(m_p[2]))
		return 1;
	return 2;
}

const double& JVector::operator[](int n) const
{
	return m_p[n];
}

double& JVector::operator[](int n)
{
	return m_p[n];
}

JVector& JVector::operator = (const JVector& v)
{
	m_p[0] = v[0];
	m_p[1] = v[1];
	m_p[2] = v[2];

	return *this;
}

JVector& JVector::operator = (const JMatrix& m)
{
	if (m.GetNCol() == 1 && m.GetNRow() == 3)
	{
		m_p[0] = m(1, 1);
		m_p[1] = m(2, 1);
		m_p[2] = m(3, 1);
	}
	return *this;
}

JVector JVector::operator + (const JVector& v) const
{
	JVector result;
	result[0] = m_p[0] + v[0];
	result[1] = m_p[1] + v[1];
	result[2] = m_p[2] + v[2];

	return result;
}

JVector JVector::operator - (const JVector& v) const
{
	JVector result;
	result[0] = m_p[0] - v[0];
	result[1] = m_p[1] - v[1];
	result[2] = m_p[2] - v[2];

	return result;
}

JVector JVector::operator * (const JVector& v) const
{
	JVector result;
	result[0] = m_p[0] * v[0];
	result[1] = m_p[1] * v[1];
	result[2] = m_p[2] * v[2];

	return result;
}

JVector JVector::operator + (const double& m) const
{
	JVector result;
	result[0] = m_p[0] + m;
	result[1] = m_p[1] + m;
	result[2] = m_p[2] + m;

	return result;
}

JVector JVector::operator - (const double& m) const
{
	JVector result;
	result[0] = m_p[0] - m;
	result[1] = m_p[1] - m;
	result[2] = m_p[2] - m;

	return result;
}

JVector JVector::operator * (const double& m) const
{
	JVector result;
	result[0] = m_p[0] * m;
	result[1] = m_p[1] * m;
	result[2] = m_p[2] * m;

	return result;
}

JVector JVector::operator / (const JVector& v) const
{
	JVector result;
	result[0] = m_p[0] / v[0];
	result[1] = m_p[1] / v[1];
	result[2] = m_p[2] / v[2];

	return result;
}

JVector JVector::operator / (const double& m) const
{
	JVector result;
	result[0] = m_p[0] / m;
	result[1] = m_p[1] / m;
	result[2] = m_p[2] / m;

	return result;
}

JVector& JVector::operator -= (const JVector& v1)
{
	m_p[0] -= v1[0];
	m_p[1] -= v1[1];
	m_p[2] -= v1[2];

	return *this;
}

JVector& JVector::operator += (const JVector& v1)
{
	m_p[0] += v1[0];
	m_p[1] += v1[1];
	m_p[2] += v1[2];

	return *this;
}

JVector& JVector::operator *= (const JVector& v1)
{
	m_p[0] *= v1[0];
	m_p[1] *= v1[1];
	m_p[2] *= v1[2];

	return *this;
}

JVector& JVector::operator /= (const JVector& v1)
{
	m_p[0] /= v1[0];
	m_p[1] /= v1[1];
	m_p[2] /= v1[2];

	return *this;
}

JVector& JVector::operator += (double s)
{
	m_p[0] += s;
	m_p[1] += s;
	m_p[2] += s;

	return *this;
}

JVector& JVector::operator -= (double s)
{
	m_p[0] -= s;
	m_p[1] -= s;
	m_p[2] -= s;

	return *this;
}

JVector& JVector::operator *= (double s)
{
	m_p[0] *= s;
	m_p[1] *= s;
	m_p[2] *= s;

	return *this;
}

JVector& JVector::operator /= (double s)
{
	m_p[0] /= s;
	m_p[1] /= s;
	m_p[2] /= s;

	return *this;
}

JVector operator + (const double& m, JVector& v)
{
	return v + m;
}

JVector operator - (const double& m, JVector& v)
{
	return v - m;
}

JVector operator * (const double& m, JVector& v)
{
	return v * m;
}

JVector operator / (const double& m, JVector& v)
{
	return v / m;
}

bool JVector::operator == (const JVector& v)
{
	return (m_p[0] == v[0] &&
		m_p[1] == v[1] &&
		m_p[2] == v[2]);
}

bool JVector::operator != (const JVector& v)
{
	return (m_p[0] != v[0] &&
		m_p[1] != v[1] &&
		m_p[2] != v[2]);
}

bool JVector::operator <= (const JVector& v)
{
	return (this->magnitude() <= v.magnitude());
}

bool JVector::operator >= (const JVector& v)
{
	return (this->magnitude() >= v.magnitude());
}

bool JVector::operator > (const JVector& v)
{
	return (this->magnitude() > v.magnitude());
}

bool JVector::operator < (const JVector& v)
{
	return (this->magnitude() < v.magnitude());
}