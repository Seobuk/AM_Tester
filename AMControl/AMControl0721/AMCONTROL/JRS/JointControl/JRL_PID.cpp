
#include "TcPch.h"
#pragma hdrstop


#include "../include/JRobotSystemDefine.h"

#include "JRL_PID.h"

JRL_PID::JRL_PID(void)
{
	SetGain(0.0, 0.0, 0.0);
	m_fErrP = m_fErrI = m_fErrD = 0.0;
}

void JRL_PID::SetGain(double Kp, double Ki, double Kd, double LAMDA)
{
	m_fKp = Kp;
	m_fKi = Ki;
	m_fKd = Kd;
	m_fLamda = LAMDA;
}

void JRL_PID::GetGain(double* Kp, double* Ki, double* Kd, double* LAMDA)
{
	*Kp = m_fKp;
	*Ki = m_fKi;
	*Kd = m_fKd;

	if (LAMDA)
		*LAMDA = m_fLamda;
}


double JRL_PID::PID(JRL_JointState& state)
{
	ComputeError(state);

	state.SetReferenceTorq(m_fErrP * m_fKp + m_fErrI * m_fKi + m_fErrD * m_fKd);
	return state.REF().TORQ();
}

double JRL_PID::PI(JRL_JointState& state)
{
	ComputeError(state);

	state.SetReferenceTorq(m_fErrP * m_fKp + m_fErrI * m_fKi);
	return state.REF().TORQ();
}

double JRL_PID::PD(JRL_JointState& state)
{
	ComputeError(state);

	state.SetReferenceTorq(m_fErrP * m_fKp + m_fErrD * m_fKd);
	return state.REF().TORQ();
}

void JRL_PID::ComputeError(JRL_JointState& state)
{
	JRL_ActuatorState pDelta;
	pDelta = state.REF() - state.ACT();

	m_fErrP = pDelta.POS();
	m_fErrI += m_fErrP * DT;
	m_fErrD = pDelta.VEL();
	m_fErrI *= (1.0 - m_fLamda);
}

const double& JRL_PID::Kp(void) const
{
	return m_fKp;
}

double& JRL_PID::Kp(void)
{
	return m_fKp;
}

const double& JRL_PID::Ki(void) const
{
	return m_fKi;
}

double& JRL_PID::Ki(void)
{
	return m_fKi;
}

const double& JRL_PID::Kd(void) const
{
	return m_fKd;
}

double& JRL_PID::Kd(void)
{
	return m_fKd;
}

const double& JRL_PID::LAMDA(void) const
{
	return m_fLamda;
}

double& JRL_PID::LAMDA(void)
{
	return m_fLamda;
}
