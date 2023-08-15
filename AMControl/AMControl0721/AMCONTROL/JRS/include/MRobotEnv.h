#pragma once
#include "./JRS/JMath/JMath.h"
//#include <math.h>

enum MRErrMsg {
	MR_NO_ERR		=	0,
	MR_FILE_ERROR	=	1,
	MR_NO_DH		=	2,
	MR_NO_DYNAMIC	=	3
};

#define TYPE_R	0	//revolut
#define TYPE_P	1	//prismatic

struct DH {
	double a;
	double alpha;
	double d;
	double theta;
	int j_type;		// 0: revolute, 1: prismatic

	double m;
	//double r[3];
	//double I[3][3];
	JMatrix r;
	JMatrix I;
	DH(void)
		: r(3, 1)
		, I(3, 3)
	{
		r.setZero();
		I.setZero();
	}
	char nType;
};

class TaskPoint {
public:
	JMatrix P;
	JMatrix O;
	JMatrix Rd;

//private:
	JMatrix m_R;
	JMatrix m_I;
	JMatrix m_quat;
	JMatrix m_omega;
	JMatrix	m_W;

public:
	TaskPoint();

	void SetEuler(double Rx, double Ry, double Rz);
	void SetEuler(JMatrix R);
	void SetEuler(void);
	void GetEuler(double & Rx, double & Ry, double & Rz);
	void GetEuler(double& Rx, double& Ry, double& Rz, JMatrix R);
	//void Map2Vec(void);
	void GetE2RMat(double Rx, double Ry, double Rz, JMatrix & R);
	JMatrix& GetE2RMat(double Rx, double Ry, double Rz);
	void GetQ2RMat(JMatrix & R);
	void MakeRMatrix(double Rx, double Ry, double Rz, JMatrix & R);
	void MakeRMatrix(double Rx, double Ry, double Rz);
	void Set_Rd(JMatrix& R);
	//void Get_Rd(JMatrix& R);
	JMatrix& Get_Rd(void);
	void Set_Pd(JMatrix& p);
	//void Get_Pd(JMatrix& p);
	JMatrix& Get_Pd(void);
	void Set_Od(JMatrix& o);
	//void Get_Od(JMatrix& o);
	JMatrix& Get_Od(void);
	//void RotFrame(JMatrix & Rd);

	void CopyFrom(TaskPoint& pt) {
		P = pt.P;
		O = pt.O;
		Rd = pt.Rd;
	}
	TaskPoint& operator = (TaskPoint& pt)
	{
		this->P = pt.P;
		this->O = pt.O;
		this->Rd = pt.Rd;

		return (*this);
	}
};

struct __KUD_KINEINFO_
{
	JMatrix 	Pos;
	JMatrix 	Vel;
	JMatrix 	Omega;
	JMatrix 	OmegaDot;
	JMatrix 	Acc;
	JMatrix 	Acc_c;
	JMatrix 	R;
	JMatrix		z;

	__KUD_KINEINFO_(void)
		:Pos(3, 1),
		Vel(3, 1),
		Omega(3, 1),
		OmegaDot(3, 1),
		Acc(3, 1),
		Acc_c(3, 1),
		R(3, 3),
		z(3, 1)
	{
		Pos.setZero();
		Vel.setZero();
		Omega.setZero();
		OmegaDot.setZero();
		Acc.setZero();
		Acc_c.setZero();
		R.setZero();
		R.setIdentity();
		z.setZero();
		z(2,0) = 1.0;
	}
};
typedef	__KUD_KINEINFO_		KineInfo;