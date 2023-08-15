
#include "TcPch.h"
#pragma hdrstop

#include "../include/JRobotSystemDefine.h"

#include "JRL_JointState.h"

#define ACT_POS		(m_cAct.POS())
#define ACT_VEL		(m_cAct.VEL())
#define ACT_ACC		(m_cAct.ACC())
#define ACT_TORQ	(m_cAct.TORQ())

#define REF_POS		(m_cRef.POS())
#define REF_VEL		(m_cRef.VEL())
#define REF_ACC		(m_cRef.ACC())
#define REF_TORQ	(m_cRef.TORQ())

JRL_JointState::JRL_JointState(void)
{

}

void JRL_JointState::InitState(double fTorq, double fPos, double fVel, double fAcc)
{
	ACT_TORQ = fTorq;
	ACT_POS = fPos;
	ACT_VEL = fVel;
	ACT_ACC = fAcc;

	m_cPrev = m_cAct;
	m_cRef = m_cAct;
}

void JRL_JointState::SetActualState(double fTorq, double fPos, double fVel, double fAcc)
{
	m_cPrev = m_cAct;
	m_cAct.SetActualState(fPos, fVel, fAcc, fTorq);
}

void JRL_JointState::SetActualState(double fTorq, double fPos, double fVel)
{
	SetActualState(fTorq, fPos, fVel, (fVel - ACT_VEL) * DT);
}

void JRL_JointState::SetActualState(double fTorq, double fPos)
{
	SetActualState(fTorq, fPos, (fPos - ACT_POS) * DT);
}

void JRL_JointState::SetReferenceTorq(double fTorq)
{
	REF_TORQ = fTorq;
}

void JRL_JointState::SetReferenceState(double fPos, double fVel, double fAcc)
{
	REF_POS = fPos;
	REF_VEL = fVel;
	REF_ACC = fAcc;
}

JRL_ActuatorState JRL_JointState::GetError(void)
{
	return m_cRef - m_cAct;
}



const JRL_ActuatorState& JRL_JointState::ACT(void) const
{
	return m_cAct;
}

JRL_ActuatorState& JRL_JointState::ACT(void)
{
	return m_cAct;
}

const JRL_ActuatorState& JRL_JointState::REF(void) const
{
	return m_cRef;
}

JRL_ActuatorState& JRL_JointState::REF(void)
{
	return m_cRef;
}

const JRL_ActuatorState& JRL_JointState::PREV(void) const
{
	return m_cPrev;
}

JRL_ActuatorState& JRL_JointState::PREV(void)
{
	return m_cPrev;
}
