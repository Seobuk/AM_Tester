

#include "TcPch.h"
#pragma hdrstop

#include "../include/JRobotSystemDefine.h"

#include "JRL_ActuatorState.h"


JRL_ActuatorState::JRL_ActuatorState(void)
{
	m_fPos = 0.0;
	m_fVel = 0.0;
	m_fAcc = 0.0;
	m_fTorq = 0.0;
}

void JRL_ActuatorState::SetActualState(double fPos, double fVel, double fAcc, double fTorq)
{
	m_fPos = fPos;
	m_fVel = fVel;
	m_fAcc = fAcc;
	m_fTorq = fTorq;
}


const double& JRL_ActuatorState::POS(void) const
{
	return m_fPos;
}

double& JRL_ActuatorState::POS(void)
{
	return m_fPos;
}

const double& JRL_ActuatorState::VEL(void) const
{
	return m_fVel;
}

double& JRL_ActuatorState::VEL(void)
{
	return m_fVel;
}

const double& JRL_ActuatorState::ACC(void) const
{
	return m_fAcc;
}

double& JRL_ActuatorState::ACC(void)
{
	return m_fAcc;
}

const double& JRL_ActuatorState::TORQ(void) const
{
	return m_fTorq;
}

double& JRL_ActuatorState::TORQ(void)
{
	return m_fTorq;
}

JRL_ActuatorState JRL_ActuatorState::operator + (const JRL_ActuatorState& cState) const
{
	JRL_ActuatorState ret;
	ret.m_fPos = m_fPos + cState.m_fPos;
	ret.m_fVel = m_fVel + cState.m_fVel;
	ret.m_fAcc = m_fAcc + cState.m_fAcc;
	ret.m_fTorq = m_fTorq + cState.m_fTorq;

	return ret;
}

JRL_ActuatorState JRL_ActuatorState::operator - (const JRL_ActuatorState& cState) const
{
	JRL_ActuatorState ret;
	ret.m_fPos = m_fPos - cState.m_fPos;
	ret.m_fVel = m_fVel - cState.m_fVel;
	ret.m_fAcc = m_fAcc - cState.m_fAcc;
	ret.m_fTorq = m_fTorq - cState.m_fTorq;

	return ret;
}
JRL_ActuatorState JRL_ActuatorState::operator * (const int n) const
{
	JRL_ActuatorState ret;
	ret.m_fPos = m_fPos * n;
	ret.m_fVel = m_fVel * n;
	ret.m_fAcc = m_fAcc * n;
	ret.m_fTorq = m_fTorq * n;

	return ret;
}

