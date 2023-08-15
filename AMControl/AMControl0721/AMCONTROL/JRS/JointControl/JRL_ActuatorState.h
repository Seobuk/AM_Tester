#pragma once
//////////////////////////////////////////////////////////////////////////
// Joint State Class
// Unit : radian, Nm, radian/sec 
// Dev by Jongwoo
// v1.0 
// 2020.04.09

class JRL_ActuatorState
{
	double m_fPos;
	double m_fVel;
	double m_fAcc;
	double m_fTorq;

public:
	JRL_ActuatorState(void);

public:

	void SetActualState(double fPos, double fVel, double fAcc, double fTorq);

public:
	const double& POS(void) const;
	double& POS(void);

	const double& VEL(void) const;
	double& VEL(void);

	const double& ACC(void) const;
	double& ACC(void);

	const double& TORQ(void) const;
	double& TORQ(void);

	JRL_ActuatorState operator + (const JRL_ActuatorState& cState) const;
	JRL_ActuatorState operator - (const JRL_ActuatorState& cState) const;
	JRL_ActuatorState operator * (const int n) const;
};


