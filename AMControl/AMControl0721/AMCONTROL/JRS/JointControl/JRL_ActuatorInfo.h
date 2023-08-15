#pragma once
//////////////////////////////////////////////////////////////////////////
// Actuator Information Class
// Dev by Jongwoo
// v1.0 
// 2020.04.09



#define ENC_TYPE_INCREMENTAL	0
#define ENC_TYPE_ABSOLUTE		1
#define ENC_TYPE_MULTI_TURN		2


class JRL_ActuatorInfo
{
private:
	char	m_nEncType;
	double	m_fEncMax;
	double	m_fEncHome;

	double	m_fGearRatio;
	double	m_fRatedTorq;

	double	m_fDirection;

private:
	double	m_fRadToCnt;
	double	m_fCntToRad;

	double	m_fNmToCnt;
	double	m_fCntToNm;
	

public:
	JRL_ActuatorInfo(void);
	JRL_ActuatorInfo(char nEncType, double fEncMax, double fEncHome, double fGearRatio, double fRatedTorq, double fDirection = 1.0);

	void InitEncoderInfo(char nEncType, double fEncMax, double fEncHome, double fGearRatio, double fRatedTorq, double fDirection = 1.0);
	void SetEncHome(double fEncHome);

	double& RadToCnt(void);
	double& CntToRad(void);

	double& NmToCnt(void);
	double& CntToNm(void);

	char	GetEncoderType(void);
	double	GetEncoderMax(void);
	double	GetEncoderHome(void);

	double	GetGearRatio(void);
	double	GetRateTorq(void);

	double	GetDirection(void);
};

