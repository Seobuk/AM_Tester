#pragma once
//////////////////////////////////////////////////////////////////////////
// Joint State Class
// Unit : None
// Dev by Jongwoo
// v1.0 
// 2020.04.09

#include "JRL_JointState.h"

class JRL_PID
{
	double m_fKp;
	double m_fKi;
	double m_fKd;
	double m_fLamda;

	double m_fErrP;
	double m_fErrI;
	double m_fErrD;

public:
	JRL_PID(void);

	void SetGain(double Kp, double Ki, double Kd, double LAMDA = 0.0);
	void GetGain(double* Kp, double* Ki, double* Kd, double * LAMDA = 0);


	double PID(JRL_JointState& state);
	double PI(JRL_JointState& state);
	double PD(JRL_JointState& state);

private:
	void ComputeError(JRL_JointState& state);

public:
	const double& Kp(void) const;
	double& Kp(void);

	const double& Ki(void) const;
	double& Ki(void);

	const double& Kd(void) const;
	double& Kd(void);

	const double& LAMDA(void) const;
	double& LAMDA(void);
};

