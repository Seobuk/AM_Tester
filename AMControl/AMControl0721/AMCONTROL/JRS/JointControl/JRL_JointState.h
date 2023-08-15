#pragma once
//////////////////////////////////////////////////////////////////////////
// Joint State Class
// Unit : radian, Nm, radian/sec 
// Dev by Jongwoo
// v1.0 
// 2020.04.09

#include "JRL_ActuatorState.h"

class JRL_JointState
{
	JRL_ActuatorState m_cAct;
	JRL_ActuatorState m_cPrev;

	JRL_ActuatorState m_cRef;

public:
	JRL_JointState(void);

	void InitState(double fTorq, double fPos, double fVel, double fAcc);

	void SetActualState(double fTorq, double fPos, double fVel, double fAcc);
	void SetActualState(double fTorq, double fPos, double fVel);
	void SetActualState(double fTorq, double fPos);

	void SetReferenceTorq(double fTorq);
	void SetReferenceState(double fPos, double fVel, double fAcc);

	JRL_ActuatorState GetError(void);


public:
	const JRL_ActuatorState& ACT(void) const;
	JRL_ActuatorState& ACT(void);

	const JRL_ActuatorState& REF(void) const;
	JRL_ActuatorState& REF(void);

	const JRL_ActuatorState& PREV(void) const;
	JRL_ActuatorState& PREV(void);
};

