//////////////////////////////////////////////////////////////////////////
// Actuator Information Class
// Dev by Jongwoo
// v1.0 
// 2020.04.09


#include "TcPch.h"
#pragma hdrstop


#include "../include/JRobotSystemDefine.h"

#include "JRL_ActuatorInfo.h"



JRL_ActuatorInfo::JRL_ActuatorInfo(void)
{
	m_nEncType = ENC_TYPE_MULTI_TURN;
	m_fEncHome = 0.0;
	m_fEncMax = 0.0;

	m_fGearRatio = 0.0;
	m_fRatedTorq = 0.0;

	m_fCntToRad = 0.0;
	m_fRadToCnt = 0.0;

	m_fCntToNm = 0.0;
	m_fNmToCnt = 0.0;

	m_fDirection = 1.0;
}

JRL_ActuatorInfo::JRL_ActuatorInfo(char nEncType, double fEncMax, double fEncHome, double fGearRatio, double fRatedTorq, double fDirection)
{
	InitEncoderInfo(nEncType, fEncMax, fEncHome, fGearRatio, fRatedTorq, fDirection);
}

void JRL_ActuatorInfo::InitEncoderInfo(char nEncType, double fEncMax, double fEncHome, double fGearRatio, double fRatedTorq, double fDirection)
{
	m_nEncType = nEncType;
	m_fEncMax = fEncMax;
	SetEncHome(fEncHome);
	m_fGearRatio = fGearRatio;
	m_fRatedTorq = fRatedTorq;

	m_fCntToRad = (2.0 * _PI) / (m_fEncMax * m_fGearRatio);
	m_fRadToCnt = 1.0 / m_fCntToRad;

	m_fCntToNm = m_fRatedTorq / 1000. * m_fGearRatio;
	m_fNmToCnt = 1.0 / m_fCntToNm;

	m_fDirection = fDirection;
}

void JRL_ActuatorInfo::SetEncHome(double fEncHome)
{
	m_fEncHome = fEncHome;
}

double& JRL_ActuatorInfo::RadToCnt(void) { return m_fRadToCnt; }
double& JRL_ActuatorInfo::CntToRad(void) { return m_fCntToRad; }

double& JRL_ActuatorInfo::NmToCnt(void) { return m_fNmToCnt; }
double& JRL_ActuatorInfo::CntToNm(void) { return m_fCntToNm; }

char	JRL_ActuatorInfo::GetEncoderType(void) { return m_nEncType; }
double	JRL_ActuatorInfo::GetEncoderMax(void) { return m_fEncMax; }
double	JRL_ActuatorInfo::GetEncoderHome(void) { return m_fEncHome; }

double	JRL_ActuatorInfo::GetGearRatio(void) { return m_fGearRatio; }
double	JRL_ActuatorInfo::GetRateTorq(void) { return m_fRatedTorq; }

double	JRL_ActuatorInfo::GetDirection(void) { return m_fDirection; }

