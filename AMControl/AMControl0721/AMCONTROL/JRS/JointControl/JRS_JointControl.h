#pragma once
//////////////////////////////////////////////////////////////////////////
// Joint Control Class
// Unit : radian, Nm, radian/sec 
// Dev by Jongwoo
// v1.0 
// 2020.04.09
#include "../include/JRobotSystemDefine.h"

#include "JRL_ActuatorInfo.h"
#include "JRL_JointState.h"
#include "JRL_PID.h"

class JRS_JointControl
{
public:
	char	m_nControlMode;
	char	m_nPIDMode;
	long	m_nDoF;

#ifdef JRS_DYNAMIC_SIZE
	JRL_ActuatorInfo*	m_pInfo;
	JRL_JointState*		m_pState;
	JRL_PID*			m_pPID;
#else
	JRL_ActuatorInfo	m_pInfo[JRS_MAX_DOF];
	JRL_JointState		m_pState[JRS_MAX_DOF];
	JRL_PID				m_pPID[JRS_MAX_DOF];
#endif

public:
//	JRS_JointControl(void);
	JRS_JointControl(long nDoF = 1);
	~JRS_JointControl();
private:
	void	ClearMemory(void);

public:
	JRS_ERR SetupDoF(long nDoF);

	JRS_ERR	SetupJointInfo(char pEncType[], double pEncMax[], double pEncHome[], double pGearRatio[], double pRatedTorq[], double pDirection[]);
	JRS_ERR	SetupJointInfo(int nIdx, char nEncType, double fEncMax, double fEncHome, double fGearRatio, double fRatedTorq, double fDirection);

	JRS_ERR	SetupJointControlMode(char nControlMode, char nPIDMode);

	JRS_ERR SetGain(double Kp[], double Ki[], double Kd[], double LAMDA[] = 0);
	JRS_ERR SetGain(int nIdx, double Kp, double Ki, double Kd, double LAMDA = 0.0);

	JRS_ERR	InitJointControl(double pActEnc[]);
	JRS_ERR InitJointControl(int nIdx, double fActEnc);


public:
	JRS_ERR UpdateActual(double pEncPos[], double pEncVel[], double pEncTorq[]);
	JRS_ERR UpdateActual(int nIdx, double fEncPos, double fEncVel, double fEncTorq);

	JRS_ERR	UpdateControl(double pRefPos[], double pRefVel[], double pRefAcc[], double pRefTorq[]);
	JRS_ERR UpdateControl(int nIdx, double fRefPos, double fRefVel, double fRefAcc, double fRefTorq);

	JRS_ERR OutputCommand(double pRefOut[]);
	JRS_ERR OutputCommand(int nIdx, double* pRefOut);

public:
	const JRL_ActuatorState&	GetAct(int nIdx);
	const JRL_ActuatorState&	GetRef(int nIdx);
	const JRL_ActuatorState&	GetPrev(int nIdx);
	
};

