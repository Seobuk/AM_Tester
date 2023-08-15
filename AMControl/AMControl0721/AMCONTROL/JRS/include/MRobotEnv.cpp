#include "../include/JRobotSystemDefine.h"

#ifdef USE_TWINCAT3
#include "TcPch.h"
#pragma hdrstop
#else
#include <memory.h>
#endif

#include "MRobotEnv.h"

//#include "Eigen/Eigen"
//#include "Eigen/Core"
//#include "Eigen/LU"
//#include "Eigen/Eigenvalues"
//#include "Eigen/unsupported/Eigen/MatrixFunctions"
//
//
//using Eigen::VectorXd;
//using Eigen::Vector3d;
//using Eigen::MatrixXd;
//using Eigen::Matrix3d;
//using Eigen::Matrix4d;
//using Eigen::Matrix3Xd;
//using Eigen::Vector4d;

using namespace std;
//using namespace Eigen;

//////////////////////////////////////////////////////////////////////////
double sign(double value)
{
	if (value < 0) return -1;
	if (value > 0) return 1;
	if (value == 0) return 0;
}

//Matrix3d & vec2skew(double x, double y, double z)
//{
//	static Matrix3d skew_matrix;
//
//	skew_matrix.setZero();
//
//	skew_matrix(0, 1) = -z;
//	skew_matrix(1, 0) = -skew_matrix(0, 1);
//
//	skew_matrix(0, 2) = y;
//	skew_matrix(2, 0) = -skew_matrix(0, 2);
//
//	skew_matrix(1, 2) = -x;
//	skew_matrix(2, 1) = -skew_matrix(1, 2);
//
//	return skew_matrix;
//}

JMatrix skew_matrix(3, 3);
JMatrix vec2skew(double x, double y, double z)//, JMatrix & skew_matrix)
{
	skew_matrix.setZero();

	skew_matrix(0, 1) = -z;
	skew_matrix(1, 0) = -skew_matrix(0, 1);

	skew_matrix(0, 2) = y;
	skew_matrix(2, 0) = -skew_matrix(0, 2);

	skew_matrix(1, 2) = -x;
	skew_matrix(2, 1) = -skew_matrix(1, 2);

	return skew_matrix;
}


void R2Q(JMatrix R, JMatrix & Q)
{
	Q.setZero();
	Q(0, 0) = 1.0;

	if ((R(0, 0) + R(1, 1) + R(2, 2) + 1.0) > 0.001)
	{
		Q(0, 0) = SQRT(R(0, 0) + R(1, 1) + R(2, 2) + 1);
	}
	if ((R(0, 0) - R(1, 1) - R(2, 2) + 1) > 0.001)
	{
		Q(1, 0) = sign(R(2, 1) - R(1, 2)) * SQRT(R(0, 0) - R(1, 1) - R(2, 2) + 1.0);
	}
	if ((R(1, 1) - R(2, 2) - R(0, 0) + 1) > 0.001)
	{
		Q(2, 0) = sign(R(0, 2) - R(2, 0)) * SQRT(R(1, 1) - R(2, 2) - R(0, 0) + 1.0);
	}
	if ((R(2, 2) - R(0, 0) - R(1, 1) + 1) > 0.001)
	{
		Q(3, 0) = sign(R(1, 0) - R(0, 1)) * SQRT(R(2, 2) - R(0, 0) - R(1, 1) + 1.0);
	}
	Q = Q * 0.5;
}

JMatrix R2Q(JMatrix R)
{
	JMatrix Q(4, 1);
	Q.setZero();
	Q(0, 0) = 1.0;

	if ((R(0, 0) + R(1, 1) + R(2, 2) + 1.0) > 0.0001)
	{
		Q(0, 0) = SQRT(R(0, 0) + R(1, 1) + R(2, 2) + 1);
	}
	if ((R(0, 0) - R(1, 1) - R(2, 2) + 1) > 0.0001)
	{
		Q(1, 0) = sign(R(2, 1) - R(1, 2)) * SQRT(R(0, 0) - R(1, 1) - R(2, 2) + 1.0);
	}
	if ((R(1, 1) - R(2, 2) - R(0, 0) + 1) > 0.0001)
	{
		Q(2, 0) = sign(R(0, 2) - R(2, 0)) * SQRT(R(1, 1) - R(2, 2) - R(0, 0) + 1.0);
	}
	if ((R(2, 2) - R(0, 0) - R(1, 1) + 1) > 0.0001)
	{
		Q(3, 0) = sign(R(1, 0) - R(0, 1)) * SQRT(R(2, 2) - R(0, 0) - R(1, 1) + 1.0);
	}
	Q = Q * 0.5;

	Q = Q / Q.norm();

	return Q;
}
//////////////////////////////////////////////////////////////////////////




TaskPoint::TaskPoint()
	: 
	//x(0.0), y(0.0), z(0.0)
	//, Ex(0.0), Ey(0.0), Ez(0.0), 
	P(3,1)
	, O(3,1)
	, Rd(3,3)
	, m_R(3,3)
	, m_quat(4,1)
	, m_omega(3,1)
	, m_I(3, 3)
	, m_W(3, 3)
{
	P.setZero();
	O.setZero();
	Rd.setIdentity();

	m_R.setIdentity();
	m_quat.setZero();
	m_quat(0, 0) = 1.0;
	m_omega.setZero();
	m_I.setIdentity();
	m_W.setIdentity();
}

void TaskPoint::SetEuler(double Rx, double Ry, double Rz)
{
	double theta = 0;

	m_R.setZero();
	m_quat.setZero();
	m_omega.setZero();

	MakeRMatrix(Rx, Ry, Rz, m_R);

	R2Q(m_R, m_quat);

	theta = ACOS(m_quat(0,0)) * 2.0;
	if (FABS(theta) > 0.001)
	{
		m_omega(0, 0) = m_quat(1, 0) / SIN(theta / 2.0);
		m_omega(1, 0) = m_quat(2, 0) / SIN(theta / 2.0);
		m_omega(2, 0) = m_quat(3, 0) / SIN(theta / 2.0);
	}
	

// 	if( bInv )
// 	{
// 		omega *= -1.0;
// 		theta = (2.0*_PI) - fabs(theta);
// 	}


	if (((m_quat(1,0) || m_quat(2,0) || m_quat(3,0))))
	{
		//Ex = theta * omega(0,0);
		//Ey = theta * omega(1,0);
		//Ez = theta * omega(2,0);
		O(0,0) = theta * m_omega(0,0);
		O(1,0) = theta * m_omega(1,0);
		O(2,0) = theta * m_omega(2,0);
	}
	else
	{
		//Ex = Ey = Ez = 0.0;
		O.setZero();
	}
}

void TaskPoint::SetEuler(JMatrix R)
{
	double theta = 0;

	m_quat.setZero();
	m_omega.setZero();
	
	m_quat = R2Q(R);

	theta = ACOS(m_quat(0,0)) * 2.0;

	if (FABS(theta) > 0.001)
	//if (FABS(theta) > 0.1)
	//if (((quat(1,0) || quat(2,0) || quat(3,0))))
	{
		m_omega(0,0) = m_quat(1,0) / SIN(theta / 2.0);
		m_omega(1,0) = m_quat(2,0) / SIN(theta / 2.0);
		m_omega(2,0) = m_quat(3,0) / SIN(theta / 2.0);
		//Ex = theta * omega(0,0);
		//Ey = theta * omega(1,0);
		//Ez = theta * omega(2,0);
		O(0,0) = theta * m_omega(0,0);
		O(1,0) = theta * m_omega(1,0);
		O(2,0) = theta * m_omega(2,0);
	}
	else
	{
		//Ex = Ey = Ez = 0.0;
		O.setZero();
	}
	//O.setZero();
}

void TaskPoint::SetEuler(void)
{
	double theta = 0;

	m_quat.setZero();
	m_omega.setZero();

	R2Q(Rd, m_quat);

	theta = ACOS(m_quat(0, 0)) * 2.0;

	if (FABS(theta) > 0.001)
		//if (FABS(theta) > 0.0001)
		//if (((quat(1,0) || quat(2,0) || quat(3,0))))
	{
		m_omega(0, 0) = m_quat(1, 0) / SIN(theta / 2.0);
		m_omega(1, 0) = m_quat(2, 0) / SIN(theta / 2.0);
		m_omega(2, 0) = m_quat(3, 0) / SIN(theta / 2.0);
		//Ex = theta * omega(0,0);
		//Ey = theta * omega(1,0);
		//Ez = theta * omega(2,0);
		O(0, 0) = theta * m_omega(0, 0);
		O(1, 0) = theta * m_omega(1, 0);
		O(2, 0) = theta * m_omega(2, 0);
	}
	else
	{
		//Ex = Ey = Ez = 0.0;
		O.setZero();
	}
	//O.setZero();

}

void TaskPoint::GetEuler(double & Rx, double & Ry, double & Rz)
{
	GetQ2RMat(m_R);

	Ry = ATAN2(-m_R(2, 0), SQRT((m_R(2, 1) * m_R(2, 1) + m_R(2, 2) * m_R(2, 2))));
	Rz = ATAN2(m_R(1, 0), m_R(0, 0));
	Rx = ATAN2(m_R(2, 1), m_R(2, 2));
}

void TaskPoint::GetEuler(double& Rx, double& Ry, double& Rz, JMatrix R)
{
	Ry = ATAN2(-R(2, 0), SQRT((R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2))));
	Rz = ATAN2(R(1, 0), R(0, 0));
	Rx = ATAN2(R(2, 1), R(2, 2));
}

//void TaskPoint::Map2Vec(void)
//{
//	P(0,0) = x;
//	P(1,0) = y;
//	P(2,0) = z;
//	O(0,0) = Ex;
//	O(1,0) = Ey;
//	O(2,0) = Ez;
//}

void TaskPoint::GetE2RMat(double Rx, double Ry, double Rz, JMatrix& R)
{
	MakeRMatrix(Rx, Ry, Rz, R);
}
JMatrix& TaskPoint::GetE2RMat(double Rx, double Ry, double Rz)
{
	MakeRMatrix(Rx, Ry, Rz, m_R);
	return m_R;
}
void TaskPoint::GetQ2RMat(JMatrix & R)
{
 	double theta = SQRT(O(0,0)*O(0,0) + O(1,0)*O(1,0) + O(2,0)*O(2,0));
 	R.setIdentity();
	m_I.setIdentity();
 
 	//if (FABS(theta) > 0.000001)
	if (FABS(theta) > 0.001)
 	{
		m_W = vec2skew(O(0,0)/theta, O(1,0)/theta, O(2,0)/theta);
 		R = m_I + m_W*SIN(theta) + m_W* m_W*(1.0-COS(theta));
 	}
}

void TaskPoint::MakeRMatrix(double Rx, double Ry, double Rz, JMatrix& R)
{
	R(0, 0) = COS(Rz) * COS(Ry);
	R(0, 1) = COS(Rz) * SIN(Ry) * SIN(Rx) - SIN(Rz) * COS(Rx);
	R(0, 2) = COS(Rz) * SIN(Ry) * COS(Rx) + SIN(Rz) * SIN(Rx);

	R(1, 0) = SIN(Rz) * COS(Ry);
	R(1, 1) = SIN(Rz) * SIN(Ry) * SIN(Rx) + COS(Rz) * COS(Rx);
	R(1, 2) = SIN(Rz) * SIN(Ry) * COS(Rx) - COS(Rz) * SIN(Rx);

	R(2, 0) = -SIN(Ry);
	R(2, 1) = COS(Ry) * SIN(Rx);
	R(2, 2) = COS(Ry) * COS(Rx);
}

void TaskPoint::MakeRMatrix(double Rx, double Ry, double Rz)
{
	m_R(0, 0) = COS(Rz) * COS(Ry);
	m_R(0, 1) = COS(Rz) * SIN(Ry) * SIN(Rx) - SIN(Rz) * COS(Rx);
	m_R(0, 2) = COS(Rz) * SIN(Ry) * COS(Rx) + SIN(Rz) * SIN(Rx);

	m_R(1, 0) = SIN(Rz) * COS(Ry);
	m_R(1, 1) = SIN(Rz) * SIN(Ry) * SIN(Rx) + COS(Rz) * COS(Rx);
	m_R(1, 2) = SIN(Rz) * SIN(Ry) * COS(Rx) - COS(Rz) * SIN(Rx);

	m_R(2, 0) = -SIN(Ry);
	m_R(2, 1) = COS(Ry) * SIN(Rx);
	m_R(2, 2) = COS(Ry) * COS(Rx);
}

void TaskPoint::Set_Rd(JMatrix& R)
{
	Rd = R;
}
// void TaskPoint::Get_Rd(JMatrix& R)
// {
// 	R = Rd;
// }
JMatrix& TaskPoint::Get_Rd(void)
{
	return Rd;
}

void TaskPoint::Set_Pd(JMatrix& p)
{
	P = p;
}

// void TaskPoint::Get_Pd(JMatrix& p)
// {
// 	p = P;
// }
JMatrix& TaskPoint::Get_Pd(void)
{
	return P;
}

void TaskPoint::Set_Od(JMatrix& o)
{
	O = o;
}
// void TaskPoint::Get_Od(JMatrix& o)
// {
// 	o = O;
// }
JMatrix& TaskPoint::Get_Od(void)
{
	return O;
}

//void TaskPoint::RotFrame(JMatrix & Rd)
//{
//	O = Rd*O;
//	//Ex = O(0,0);
//	//Ey = O(1,0);
//	//Ez = O(2,0);
//}