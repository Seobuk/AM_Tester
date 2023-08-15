#include "TcPch.h"
#pragma hdrstop

#include "../include/JRobotSystemDefine.h"


#include "JRS_JointControl.h"

#include <string.h>

// JRS_JointControl::JRS_JointControl(void)
// {
// 	m_nDoF = 0;
// 	m_nControlMode = -1;
// 	m_nPIDMode = -1;
// 	m_pInfo = NULL;
// 	m_pState = NULL;
// 	m_pPID = NULL;
// }

JRS_JointControl::JRS_JointControl(long nDoF)
{
	m_nDoF = nDoF;
	m_nControlMode = -1;
	m_nPIDMode = -1;
#ifdef JRS_DYNAMIC_SIZE
	m_pInfo = NULL;
	m_pState = NULL;
	m_pPID = NULL;
#endif

	SetupDoF(m_nDoF);
}

JRS_JointControl::~JRS_JointControl()
{
	ClearMemory();
}

void	JRS_JointControl::ClearMemory(void)
{
#ifdef JRS_DYNAMIC_SIZE
	if ( m_pInfo )
		delete[] m_pInfo;
	if ( m_pState )
		delete[]  m_pState;
	if (m_pPID)
		delete[] m_pPID;

	m_pInfo = NULL;
	m_pState = NULL;
	m_pPID = NULL;
#else
	memset(m_pInfo, NULL, sizeof(JRL_ActuatorInfo) * JRS_MAX_DOF);
	memset(m_pState, NULL, sizeof(JRL_ActuatorState) * JRS_MAX_DOF);
	memset(m_pPID, NULL, sizeof(JRL_PID) * JRS_MAX_DOF);
#endif
	m_nDoF = 0;
}


JRS_ERR JRS_JointControl::SetupDoF(long nDoF)
{
#ifdef JRS_DYNAMIC_SIZE
	if (nDoF < 1)
		return ERR_DOF;
#else
	if (nDoF < 1 || nDoF > JRS_MAX_DOF)
		return ERR_DOF;
#endif

	ClearMemory();

	m_nDoF = nDoF;

#ifdef JRS_DYNAMIC_SIZE
	m_pInfo = new JRL_ActuatorInfo[m_nDoF];
	m_pState = new JRL_JointState[m_nDoF];
	m_pPID = new JRL_PID[m_nDoF];
#endif

	memset(m_pInfo, NULL, sizeof(JRL_ActuatorInfo) * m_nDoF);
	memset(m_pState, NULL, sizeof(JRL_JointState) * m_nDoF);
	memset(m_pPID, NULL, sizeof(JRL_PID) * m_nDoF);

	return ERR_NONE;
}

JRS_ERR	JRS_JointControl::SetupJointInfo(char pEncType[], double pEncMax[], double pEncHome[], double pGearRatio[], double pRatedTorq[], double pDirection[])
{
	if (m_nDoF < 1 )
		return ERR_DOF;

	for (int i = 0; i < m_nDoF; i++)
		SetupJointInfo(i, pEncType[i], pEncMax[i], pEncHome[i], pGearRatio[i], pRatedTorq[i], pDirection[i]);

	return ERR_NONE;
}

JRS_ERR	JRS_JointControl::SetupJointInfo(int nIdx, char nEncType, double fEncMax, double fEncHome, double fGearRatio, double fRatedTorq, double fDirection)
{
	if ( m_nDoF < 1 || nIdx < 0 || nIdx >= m_nDoF )
		return ERR_DOF;

	m_pInfo[nIdx].InitEncoderInfo(nEncType, fEncMax, fEncHome, fGearRatio, fRatedTorq, fDirection);
	return ERR_NONE;
}

JRS_ERR	JRS_JointControl::SetupJointControlMode(char nControlMode, char nPIDMode)
{
	m_nControlMode = nControlMode;
	m_nPIDMode = nPIDMode;

	return ERR_NONE;
}

JRS_ERR JRS_JointControl::SetGain(double Kp[], double Ki[], double Kd[], double LAMDA[])
{
	if (m_nDoF < 1)
		return ERR_DOF;

	for (int i = 0; i < m_nDoF; i++) {
		if (LAMDA)
			SetGain(i, Kp[i], Ki[i], Kd[i], LAMDA[i]);
		else
			SetGain(i, Kp[i], Ki[i], Kd[i]);
	}

	return ERR_NONE;	
}

JRS_ERR JRS_JointControl::SetGain(int nIdx, double Kp, double Ki, double Kd, double LAMDA)
{
	if (m_nDoF < 1 || nIdx < 0 || nIdx >= m_nDoF)
		return ERR_DOF;

	m_pPID[nIdx].SetGain(Kp, Kd, Ki, LAMDA);
	return ERR_NONE;
}

JRS_ERR	JRS_JointControl::InitJointControl(double pActEnc[])
{
	if (m_nDoF < 1)
		return ERR_DOF;

	for (int i = 0; i < m_nDoF; i++)
		InitJointControl(i, pActEnc[i]);

	return ERR_NONE;
}

JRS_ERR JRS_JointControl::InitJointControl(int nIdx, double fActEnc)
{
	if (m_nDoF < 1 || nIdx < 0 || nIdx >= m_nDoF)
		return ERR_DOF;

	// Initialize Coding
	double fInitPosRad;
	fInitPosRad = (fActEnc - m_pInfo[nIdx].GetEncoderHome()) * m_pInfo[nIdx].CntToRad();
	fInitPosRad *= m_pInfo[nIdx].GetDirection();

	m_pState[nIdx].InitState(0.0, fInitPosRad, 0.0, 0.0);

	return ERR_NONE;
}

JRS_ERR JRS_JointControl::UpdateActual(double pEncPos[], double pEncVel[], double pEncTorq[])
{
	if (m_nDoF < 1)
		return ERR_DOF;

	for (int i = 0; i < m_nDoF; i++)
		UpdateActual(i, pEncPos[i], pEncVel[i], pEncTorq[i]);

	return ERR_NONE;
}

JRS_ERR JRS_JointControl::UpdateActual(int nIdx, double fEncPos, double fEncVel, double fEncTorq)
{
	if (m_nDoF < 1 || nIdx < 0 || nIdx >= m_nDoF)
		return ERR_DOF;

	
	double fActPos, fActVel, fActTorq;
	fActPos = (fEncPos - m_pInfo[nIdx].GetEncoderHome()) * m_pInfo[nIdx].CntToRad() * m_pInfo[nIdx].GetDirection();
	fActVel = fEncVel * m_pInfo[nIdx].CntToRad() * m_pInfo[nIdx].GetDirection();
	fActTorq = fEncTorq * m_pInfo[nIdx].CntToNm() * m_pInfo[nIdx].GetDirection();

	m_pState[nIdx].SetActualState(fActTorq, fActPos, fActVel);

	return ERR_NONE;
}

JRS_ERR	JRS_JointControl::UpdateControl(double pRefPos[], double pRefVel[], double pRefAcc[], double pRefTorq[])
{
	if (m_nDoF < 1)
		return ERR_DOF;

	JRS_ERR eErr;
	double fPos, fVel, fAcc, fTorq;
	for (int i = 0; i < m_nDoF; i++) {
		fPos = pRefPos ? pRefPos[i] : JRS_JCTRL_NONE;
		fVel = pRefVel ? pRefVel[i] : JRS_JCTRL_NONE;
		fAcc = pRefAcc ? pRefAcc[i] : JRS_JCTRL_NONE;
		fTorq = pRefTorq ? pRefTorq[i] : JRS_JCTRL_NONE;

		eErr = UpdateControl(i, fPos, fVel, fAcc, fTorq);
		if (eErr != ERR_NONE)
			return eErr;
	}

	return ERR_NONE;
}

JRS_ERR JRS_JointControl::UpdateControl(int nIdx, double fRefPos, double fRefVel, double fRefAcc, double fRefTorq)
{
	if (m_nDoF < 1 || nIdx < 0 || nIdx >= m_nDoF)
		return ERR_DOF;


	if (m_nControlMode == JRS_JCTRL_MODE_TORQ) {
		// Direct Torq Mode
		m_pState[nIdx].SetReferenceState(m_pState[nIdx].ACT().POS(), m_pState[nIdx].ACT().VEL(), m_pState[nIdx].ACT().ACC());
		m_pState[nIdx].SetReferenceTorq(fRefTorq);
		return ERR_NONE;
	}

	JRL_ActuatorState sDelta;
	sDelta = m_pState[nIdx].ACT() - m_pState[nIdx].PREV();

	double fPos, fVel, fAcc;
	if (fRefPos == JRS_JCTRL_NONE)
		fPos = m_pState[nIdx].REF().POS() + fRefVel * DT;
	else
		fPos = fRefPos;

	if (fRefVel == JRS_JCTRL_NONE)
		fVel = sDelta.POS() / DT;
	else
		fVel = fRefVel;

	if (fRefAcc == JRS_JCTRL_NONE)
		fAcc = sDelta.VEL() / DT;
	else
		fAcc = fRefAcc;

	m_pState[nIdx].SetReferenceState(fPos, fVel, fAcc);

	switch (m_nControlMode) {
		case JRS_JCTRL_MODE_POS:
		case JRS_JCTRL_MODE_VEL:
			m_pState[nIdx].SetReferenceTorq(m_pState[nIdx].ACT().TORQ());
			break;
		case JRS_JCTRL_MODE_TORQ_POS:
			m_pPID[nIdx].PID(m_pState[nIdx]);
			break;
		case JRS_JCTRL_MODE_TORQ_VEL:
			return ERR_JCTRL_MISS;
			break;
		case JRS_JCTRL_MODE_TORQ:
			break;
		default:
			return ERR_JCTRL_MODE;
	}

	return ERR_NONE;
}

JRS_ERR JRS_JointControl::OutputCommand(double pRefOut[])
{
	if (m_nDoF < 1)
		return ERR_DOF;

	JRS_ERR eErr;
	for (int i = 0; i < m_nDoF; i++) {
		eErr = OutputCommand(i, &pRefOut[i]);
		if (eErr != ERR_NONE)
			return eErr;
	}

	return ERR_NONE;
}

JRS_ERR JRS_JointControl::OutputCommand(int nIdx, double* pRefOut)
{
	if (m_nDoF < 1 || nIdx < 0 || nIdx >= m_nDoF)
		return ERR_DOF;

	switch (m_nControlMode) {
		case JRS_JCTRL_MODE_POS:
			*pRefOut = (m_pState[nIdx].REF().POS() * m_pInfo[nIdx].RadToCnt() * m_pInfo[nIdx].GetDirection()) + m_pInfo[nIdx].GetEncoderHome();
			break;
		case JRS_JCTRL_MODE_VEL:
			*pRefOut = m_pState[nIdx].REF().VEL() * m_pInfo[nIdx].RadToCnt() * m_pInfo[nIdx].GetDirection();
			break;
		case JRS_JCTRL_MODE_TORQ_POS:
		case JRS_JCTRL_MODE_TORQ:
			*pRefOut = m_pState[nIdx].REF().TORQ() * m_pInfo[nIdx].NmToCnt() * m_pInfo[nIdx].GetDirection();
			break;
		case JRS_JCTRL_MODE_TORQ_VEL:
			return ERR_JCTRL_MISS;
			break;
		default:
			return ERR_JCTRL_MODE;
	}

	return ERR_NONE;
}

const JRL_ActuatorState& JRS_JointControl::GetAct(int nIdx)
{
	return m_pState[nIdx].ACT();
}

const JRL_ActuatorState& JRS_JointControl::GetRef(int nIdx)
{
	return m_pState[nIdx].REF();
}

const JRL_ActuatorState& JRS_JointControl::GetPrev(int nIdx)
{
	return m_pState[nIdx].PREV();
}
