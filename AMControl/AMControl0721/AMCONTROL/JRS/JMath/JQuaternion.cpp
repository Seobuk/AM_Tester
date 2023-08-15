#include "TcPch.h"
#pragma hdrstop

#include "../include/JRobotSystemDefine.h"

#include "JMath.h"

template <class T>
T JClamp(const T &val, const T &min, const T &max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}


JQuaternion::JQuaternion(void)
	: m_s(1.0)
	, m_v(3, 1)
{
	m_v.setZero();
}

JQuaternion::JQuaternion(double r, JMatrix v)
	: m_s(1.0)
	, m_v(v)
{


}

JQuaternion::JQuaternion(JMatrix R)
	:m_s(1.0)
	, m_v(3, 1)
{
	SetRotationMatrix(R);
}

//JQuaternion::JQuaternion(const JQuaternion &Cpy)
//{
//	*this = Cpy;
//}

JQuaternion::~JQuaternion(void)
{

}


void JQuaternion::SetQuaternion(double theta, JMatrix axis)
{
	m_s = theta;
	m_v = axis;
}


// operator const double*() const { return &m_s; }
// operator double*() { return &m_s; }


void JQuaternion::scale(double s)
{
	m_s *= s;
	m_v *= s;
}

void JQuaternion::normalize()
{
	JMatrix m_sqr(1, 1);
	m_sqr = m_v.Transpose() * m_v;

	double lengthSqr = m_s * m_s + m_sqr(0);
	if (lengthSqr != 0)
	{
		scale(1.0 / SQRT(lengthSqr));
	}
}

double JQuaternion::norm(void)
{
	JMatrix m_sqr(1, 1);
	m_sqr = m_v.Transpose() * m_v;
	return m_s * m_s + m_sqr(1);
}

void JQuaternion::GetRotationMatrix(double R[3][3]) const
{
	//double tx  = 2.0*m_v(0);
 //   double ty  = 2.0*m_v(1);
 //   double tz  = 2.0*m_v(2);
 //   double twx = tx*m_s;
 //   double twy = ty*m_s;
 //   double twz = tz*m_s;
 //   double txx = tx*m_v(0);
 //   double txy = ty*m_v(0);
 //   double txz = tz*m_v(0);
 //   double tyy = ty*m_v(1);
 //   double tyz = tz*m_v(1);
 //   double tzz = tz*m_v(2);

 //   R[0][0] = 1.0-(tyy+tzz);
 //   R[1][0] = txy+twz;
 //   R[2][0] = txz-twy;
 //   R[0][1] = txy-twz;
 //   R[1][1] = 1.0-(txx+tzz);
 //   R[2][1] = tyz+twx;
 //   R[0][2] = txz+twy;
 //   R[1][2] = tyz-twx;
 //   R[2][2] = 1.0-(txx+tyy);
	R[0][0] = 2.0 * (m_s * m_s + m_v(0) * m_v(0)) - 1.0;
	R[0][1] = 2.0 * (m_v(0) * m_v(1) - m_s * m_v(2));
	R[0][2] = 2.0 * (m_v(0) * m_v(2) + m_s * m_v(1));
	R[1][0] = 2.0 * (m_v(0) * m_v(1) + m_s * m_v(2));
	R[1][1] = 2.0 * (m_s * m_s + m_v(1) * m_v(1)) - 1.0;
	R[1][2] = 2.0 * (m_v(1) * m_v(2) - m_s * m_v(0));
	R[2][0] = 2.0 * (m_v(0) * m_v(2) - m_s * m_v(1));
	R[2][1] = 2.0 * (m_v(1) * m_v(2) + m_s * m_v(0));
	R[2][2] = 2.0 * (m_s * m_s + m_v(2) * m_v(2)) - 1.0;
}

void JQuaternion::SetRotationMatrix(const double R[3][3])
{
	double trace = R[0][0] + R[1][1] + R[2][2];
	double root;

	double w, x, y, z;

	//if ( trace > 0.0 )
	//{
	//    // |w| > 1/2, may as well choose w > 1/2
	//    root = SQRT(trace+1.0);  // 2w
	//    w = 0.5*root;
	//    root = 0.5/root;  // 1/(4w)
	//    x = (R[1][2]-R[2][1])*root;
	//    y = (R[2][0]-R[0][2])*root;
	//    z = (R[0][1]-R[1][0])*root;
	//}
	//else
	//{
	//    // |w| <= 1/2
	//    static int next[3] = { 1, 2, 0 };
	//    int i = 0;
	//    if ( R[1][1] > R[0][0] )
	//        i = 1;
	//    if ( R[2][2] > R[i][i] )
	//        i = 2;
	//    int j = next[i];
	//    int k = next[j];

	//    root = SQRT(R[i][i]-R[j][j]-R[k][k]+1.0);
	//    double* quat[3] = { &x, &y, &z };
	//    *quat[i] = 0.5*root;
	//    root = 0.5/root;
	//    w = (R[j][k]-R[k][j])*root;
	//    *quat[j] = (R[i][j]+R[j][i])*root;
	//    *quat[k] = (R[i][k]+R[k][i])*root;
	//}

	w = 1.0;
	x = 0.0;
	y = 0.0;
	z = 0.0;
	if (trace + 1.0 > 0.0)
		w = 0.5 * SQRT(trace + 1.0);
	if (R[0][0] - R[1][1] - R[2][2] + 1.0 > 0.0)
	{
		if (R[2][1] - R[1][2] > 0)
			x = 0.5 * SQRT(R[0][0] - R[1][1] - R[2][2] + 1.0);
		else
			x = -0.5 * SQRT(R[0][0] - R[1][1] - R[2][2] + 1.0);
	}
	if (R[1][1] - R[2][2] - R[0][0] + 1.0 > 0.0)
	{
		if (R[0][2] - R[2][0] > 0)
			y = 0.5 * SQRT(R[1][1] - R[2][2] - R[0][0] + 1.0);
		else
			y = -0.5 * SQRT(R[1][1] - R[2][2] - R[0][0] + 1.0);
	}
	if (R[2][2] - R[0][0] - R[1][1] + 1.0 > 0.0)
	{
		if (R[1][0] - R[0][1] > 0)
			z = 0.5 * SQRT(R[2][2] - R[0][0] - R[1][1] + 1.0);
		else
			z = -0.5 * SQRT(R[2][2] - R[0][0] - R[1][1] + 1.0);
	}

	m_v(0) = x;
	m_v(1) = y;
	m_v(2) = z;
	m_s = w;
}

void JQuaternion::GetRotationMatrix(JMatrix& rotMat) const
{
	double r[3][3];
	GetRotationMatrix(r);

	int i;
	int j;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			rotMat[i][j] = r[i][j];
		}
	}
}

void JQuaternion::SetRotationMatrix(const JMatrix& rotMat)
{
	double r[3][3];

	int i;
	int j;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			r[i][j] = rotMat[i][j];
		}
	}

	SetRotationMatrix(r);
}

// Converts to/from axis/angle representation and quaternion.
// Angle is specified in radians
void JQuaternion::GetAxisAngle(JMatrix& axis, double& angle)
{
	JMatrix m_sqr(1, 1);
	m_sqr = m_v.Transpose() * m_v;
	double lenSquared = m_sqr(0);
	double temp_ms;

	if (lenSquared > 0.0)
	{
		//assert(m_s >= -1.0 && m_s <= 1.0);

		if (m_s < -1.0) temp_ms = -1.0;
		if (m_s > 1.0) temp_ms = 1.0;
		angle = 2.0 * ACOS(temp_ms);
		double invlen = 1.0 / SQRT(lenSquared);
		axis = m_v * invlen;
	}
	else
	{
		// If angle is 0 so axis doesn't matter, pick any
		angle = 0.0;
		axis(0) = 1.0;
		axis(1) = 0.0;
		axis(2) = 0.0;
	}
}

void JQuaternion::SetAxisAngle(const JMatrix& axis, double angle)
{
	JMatrix t = axis;
	JMatrix m_sqr(1, 1);
	m_sqr = t.Transpose() * t;
	t = t / (SQRT(m_sqr(0)));

	double halfAngle = angle * 0.5;
	double sinTheta = SIN(halfAngle);
	m_s = COS(halfAngle);
	m_v = sinTheta * t;
}


JQuaternion& JQuaternion::operator = (JQuaternion& q)
{
	this->m_s = q.m_s;
	this->m_v = q.m_v;
	return *this;
}

JQuaternion JQuaternion::operator * (const JQuaternion& q)
{
	double s1 = this->m_s;
	double s2 = q.m_s;
	JMatrix v1 = this->m_v;
	JMatrix v2 = q.m_v;

	JQuaternion Temp;
	JMatrix m_sqr(1, 1);
	m_sqr = v1.Transpose() * v2;
	Temp.m_s = s1 * s2 - m_sqr(0);
	Temp.m_v = s1 * v2 + s2 * v1 + v1.cross(v2);

	return Temp;
}

JQuaternion JQuaternion::operator * (const JMatrix& v)
{
	double s1 = this->m_s;
	double s2 = 0.0;
	JMatrix v1 = this->m_v;
	JMatrix v2 = v;

	JQuaternion Temp;
	JMatrix m_sqr(1, 1);
	m_sqr = v1.Transpose() * v2;
	Temp.m_s = s1 * s2 - m_sqr(0);
	Temp.m_v = s1 * v2 + s2 * v1 + v1.cross(v2);

	return Temp;
}

JQuaternion JQuaternion::operator * (const double& n)
{
	JQuaternion Temp;
	Temp.m_s = this->m_s * n;
	Temp.m_v = this->m_v * n;
	return Temp;
}

JQuaternion operator * (const JMatrix& v, JQuaternion& q)
{
	return q * v;
}

JQuaternion operator * (const double& n, JQuaternion& q)
{
	return q * n;
}