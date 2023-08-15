#ifndef _JRS_TRAJECTORY_5TH_POLYNOMIAL_H_
#define _JRS_TRAJECTORY_5TH_POLYNOMIAL_H_


#include "../include/JRobotSystemDefine.h"

#include "../JMath/JMath.h"


class JRS_Trajectory {
	char m_nTrajMode;

	double t0;
	double t1;

	double p0;
	double p1;
	double v0;
	double v1;
	double a0;
	double a1;

public:
	JRS_Trajectory(void);

	JRS_ERR SetMode(char nTrajMode);
	// time unit sec & Trapezoid don't use T1
	JRS_ERR SetTrajectory(double fT0, double fT1, double fP0, double fP1, double fV0 = 0.0, double fV1 = 0.0, double fA0 = 0.0, double fA1 = 0.0);
	// only 5th trajectory
	JRS_ERR SetSlowStop(double fSlowTime = 0.5);

	void GetTrajectory(double t, double * pos, double * vel = 0, double * acc = 0);
	void GetLastTrajectory(double * pos, double * vel = 0, double * acc = 0);


//////////////////////////////////////////////////////////////////////////
// 5th polynomial trajectory function & variable
private:
	JMatrix m_c5thExp;
	JMatrix m_c5thDes;
	JMatrix m_c5thConst;

	double last_t;
	double last_p;
	double last_v;
	double last_a;

private:
	void Set5Th(double fT0, double fT1, double fP0, double fP1, double fV0 = 0.0, double fV1 = 0.0, double fA0 = 0.0, double fA1 = 0.0);
	void Get5th(double t, double* pos, double* vel = 0, double* acc = 0);

	void ComputeConst5th(void);
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// Trapezoid trajectory function & variable
private:
	double m_fTrapezoidMaxVel;
	double m_fTrapezoidAcc;

	double m_fTrapezoidT0;
	double m_fTrapezoidT1;
	double m_fTrapezoidT2;
	double m_fTrapezoidT3;

	double m_fTrapezoidT01;
	double m_fTrapezoidT12;
	double m_fTrapezoidT23;

	double m_fTrapezoidActVelMax;
	double m_fTrapezoidActAcc;

public:
	// time unit sec
	void SetTrapezoidVelAcc(double vel, double acc);
	// time unit sec
	void SetTrapezoidVel(double vel);
	// time unit sec
	void SetTrapezoidAcc(double acc);

private:
	// time unit sec & Don't use T0, T1
	void SetTrapezoid(double fT0, double fT1, double fP0, double fP1, double fV0 = 0.0, double fV1 = 0.0, double fA0 = 0.0, double fA1 = 0.0);
	void GetTrapezoid(double t, double* pos, double* vel = 0, double* acc = 0);
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// Linear trajectory function & variable
private:
	double m_fLinearDeltaT;
	double m_fLinearDeltaP;
	double m_fLinearConst;
public:
	void SetLinear(double fT0, double fT1, double fP0, double fP1);
	void GetLinear(double t, double* pos, double* vel = 0, double* acc = 0);
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// Sine trajectory function & variable
private:
	double m_fSineTf;
	double m_fSineT;
	double m_fSineOmega;
	double m_fSineMag;
	double m_fSineFreq;
	double m_fSineCycle;
public:
	void SetSine(double fMAG = 0, double fFREQ = 0, double fCYCLE = 0);
	void GetSine(double t, double* pos, double* vel = 0, double* acc = 0);
};


#endif
