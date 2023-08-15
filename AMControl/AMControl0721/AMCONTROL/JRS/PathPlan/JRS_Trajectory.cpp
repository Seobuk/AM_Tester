
#include "TcPch.h"
#pragma hdrstop


#include "../include/JRobotSystemDefine.h"

#include "JRS_Trajectory.h"

#define DEFAULT_T0	0.0
#define DEFAULT_T1	0.0

#define DEFAULT_P0	0.0
#define DEFAULT_P1	0.0



JRS_Trajectory::JRS_Trajectory(void)
	: m_c5thExp(6, 6)
	, m_c5thDes(6, 1)
	, m_c5thConst(6, 1)
{
	// 	m_c5thExp = JMatrix::JMatrix(6, 6);
	// 	m_c5thDes = JMatrix::JMatrix(6, 1);
	// 	m_c5thConst = JMatrix::JMatrix(6, 1);

	SetTrajectory(JRS_TRAJ_5TH, DEFAULT_P0, DEFAULT_T1, DEFAULT_P0, DEFAULT_P1);
}

JRS_ERR JRS_Trajectory::SetMode(char nTrajMode)
{
	
	switch (nTrajMode) {
	case JRS_TRAJ_5TH:
	case JRS_TRAJ_TRAPEZOID:
	case JRS_TRAJ_LINEAR:
	case JRS_TRAJ_SIN:
		break;
	default:
		return ERR_TRAJ_TYPE;
	}

	m_nTrajMode = nTrajMode;

	return ERR_NONE;
}

JRS_ERR JRS_Trajectory::SetTrajectory(double fT0, double fT1, double fP0, double fP1, double fV0, double fV1, double fA0, double fA1)
{
	t0 = fT0;
	t1 = fT1;

	p0 = fP0;
	p1 = fP1;

	v0 = fV0;
	v1 = fV1;

	a0 = fA0;
	a1 = fA1;

	last_t = fT0;
	last_p = fP0;
	last_v = fV0;
	last_a = fA0;

	switch (m_nTrajMode) {
	case JRS_TRAJ_5TH:
		Set5Th(fT0, fT1, fP0, fP1, fV0, fV1, fA0, fA1);
		break;
	case JRS_TRAJ_TRAPEZOID:
		SetTrapezoid(fT0, fT1, fP0, fP1, fV0, fV1, fA0, fA1);
		break;
	case JRS_TRAJ_LINEAR:
		SetLinear(fT0, fT1, fP0, fP1);
		break;
	case JRS_TRAJ_SIN:
		SetSine(fP1, fT1, 100.0); //임시 코드 (magnitude / frequency/ cycle)
		break;
	default:
		return ERR_TRAJ_TYPE;
	}

	return ERR_NONE;
}

JRS_ERR JRS_Trajectory::SetSlowStop(double fSlowTime)
{
	if (m_nTrajMode != JRS_TRAJ_5TH)
		return ERR_TRAJ_TYPE;

	JMatrix cSSExp(4, 4);
	JMatrix cSSDes(4, 1);
	JMatrix cSSConst(4, 1);

	cSSDes(0, 0) = last_p;	// Position Actual
	cSSDes(1, 0) = last_v;	// Velocity Actual
	cSSDes(2, 0) = 0.0;		// Velocity Desired
	cSSDes(3, 0) = last_a;	// Acceleration Actual

	double t0_3, t0_2, t0_1;
	double t1_2, t1_1;

	t0_1 = last_t;
	t0_2 = t0_1 * t0_1;
	t0_3 = t0_2 * t0_1;

	t1_1 = last_t+fSlowTime;
	t1_2 = t1_1 * t1_1;

	cSSExp(0, 0) = t0_3;		cSSExp(0, 1) = t0_2;		cSSExp(0, 2) = t0_1;	cSSExp(0, 3) = 1.0;
	cSSExp(1, 0) = 3.0 * t0_2;	cSSExp(1, 1) = 2.0 * t0_1;	cSSExp(1, 2) = 1.0;		cSSExp(1, 3) = 0.0;
	cSSExp(2, 0) = 3.0 * t1_2;	cSSExp(2, 1) = 2.0 * t1_1;	cSSExp(2, 2) = 1.0;		cSSExp(2, 3) = 0.0;
	cSSExp(3, 0) = 6.0 * t0_1;	cSSExp(3, 1) = 2.0;			cSSExp(3, 2) = 0.0;		cSSExp(3, 3) = 0.0;


	cSSConst = cSSExp.GetInverse() * cSSDes;	// 5th poly const c, d, e, f

	m_c5thConst(0, 0) = 0.0;
	m_c5thConst(1, 0) = 0.0;
	m_c5thConst(2, 0) = cSSConst(0, 0);
	m_c5thConst(3, 0) = cSSConst(1, 0);
	m_c5thConst(4, 0) = cSSConst(2, 0);
	m_c5thConst(5, 0) = cSSConst(3, 0);

	this->t1 = last_t + fSlowTime;

	return ERR_NONE;
}

void JRS_Trajectory::GetTrajectory(double t, double * pos, double * vel, double * acc)
{
	last_t = t;
	switch (m_nTrajMode ) {
	case JRS_TRAJ_5TH:
		Get5th(t, pos, vel, acc);
		break;
	case JRS_TRAJ_TRAPEZOID:
		GetTrapezoid(t, pos, vel, acc);
		break;
	case JRS_TRAJ_LINEAR:
		GetLinear(t, pos, vel, acc);
		break;
	case JRS_TRAJ_SIN:
		GetSine(t, pos, vel, acc);
		break;
	}
}

void JRS_Trajectory::GetLastTrajectory(double * pos, double * vel, double * acc)
{
	*pos = last_p;

	if( vel != NULL )
		*vel = last_v;

	if( acc != NULL )
		*acc = last_a;
}

void JRS_Trajectory::Set5Th(double fT0, double fT1, double fP0, double fP1, double fV0, double fV1, double fA0, double fA1)
{
	ComputeConst5th();
}

void JRS_Trajectory::Get5th(double t, double* pos, double* vel, double* acc)
{
	if (this->t0 - this->t1 != 0.0) {
		if (t < t0)
			t = t0;
		else if (t > t1)
			t = t1;

		double t2 = t * t;
		double t3 = t * t2;
		double t4 = t * t3;
		double t5 = t * t4;

		double a = m_c5thConst(0, 0);
		double b = m_c5thConst(1, 0);
		double c = m_c5thConst(2, 0);
		double d = m_c5thConst(3, 0);
		double e = m_c5thConst(4, 0);
		double f = m_c5thConst(5, 0);

		last_p = (a * t5) + (b * t4) + (c * t3) + (d * t2) + (e * t) + f;
		last_v = (5.0 * a * t4) + (4.0 * b * t3) + (3.0 * c * t2) + (2.0 * d * t) + e;
		last_a = (20.0 * a * t3) + (12.0 * b * t2) + (6.0 * c * t) + (2.0 * d);
	}

	GetLastTrajectory(pos, vel, acc);
}

void JRS_Trajectory::ComputeConst5th(void)
{
	if (this->t0 - this->t1 == 0) {
		m_c5thConst.setZero();
		return;
	}
	double t0[6];
	double t1[6];

	t0[0] = 1.0;
	t1[0] = 1.0;

	for (int i = 1; i < 6; i++) {
		t0[i] = t0[i - 1] * this->t0;
		t1[i] = t1[i - 1] * this->t1;
	}




	m_c5thExp(0, 0) = t0[5];		m_c5thExp(0, 1) = t0[4];		m_c5thExp(0, 2) = t0[3];		m_c5thExp(0, 3) = t0[2];		m_c5thExp(0, 4) = t0[1];	m_c5thExp(0, 5) = 1.0;
	m_c5thExp(1, 0) = t1[5];		m_c5thExp(1, 1) = t1[4];		m_c5thExp(1, 2) = t1[3];		m_c5thExp(1, 3) = t1[2];		m_c5thExp(1, 4) = t1[1];	m_c5thExp(1, 5) = 1.0;
	m_c5thExp(2, 0) = 5.0 * t0[4];	m_c5thExp(2, 1) = 4.0 * t0[3];	m_c5thExp(2, 2) = 3.0 * t0[2];	m_c5thExp(2, 3) = 2.0 * t0[1];	m_c5thExp(2, 4) = 1.0;		m_c5thExp(2, 5) = 0.0;
	m_c5thExp(3, 0) = 5.0 * t1[4];	m_c5thExp(3, 1) = 4.0 * t1[3];	m_c5thExp(3, 2) = 3.0 * t1[2];	m_c5thExp(3, 3) = 2.0 * t1[1];	m_c5thExp(3, 4) = 1.0;		m_c5thExp(3, 5) = 0.0;
	m_c5thExp(4, 0) = 20.0 * t0[3];	m_c5thExp(4, 1) = 12.0 * t0[2];	m_c5thExp(4, 2) = 6.0 * t0[1];	m_c5thExp(4, 3) = 2.0;			m_c5thExp(4, 4) = 0.0;		m_c5thExp(4, 5) = 0.0;
	m_c5thExp(5, 0) = 20.0 * t1[3];	m_c5thExp(5, 1) = 12.0 * t1[2];	m_c5thExp(5, 2) = 6.0 * t1[1];	m_c5thExp(5, 3) = 2.0;			m_c5thExp(5, 4) = 0.0;		m_c5thExp(5, 5) = 0.0;

	m_c5thDes(0, 0) = p0;
	m_c5thDes(1, 0) = p1;
	m_c5thDes(2, 0) = v0;
	m_c5thDes(3, 0) = v1;
	m_c5thDes(4, 0) = a0;
	m_c5thDes(5, 0) = a1;

	m_c5thConst = m_c5thExp.GetInverse() * m_c5thDes;

	// 	a = a0 / (2.0 * t01_3) + (6.0 * p0) / t01_5 - (6.0 * p1) / t01_5 - (3.0 * v0) / t01_4 - v1 / (2.0 * t01_3) - (3.0 * v1) / t01_4;
	// 	b = (v1 * ((3.0 * t0) / 2.0 + 51)) / t01_3 - (a0 * (t0 + (3.0 * t1) / 2)) / t01_3 - (p0 * (15.0 * t0 + 15.0 * t1)) / t01_5 + (p1 * (15.0 * t0 + 15.0 * t1)) / t01_5 + (v0 * (7.0 * t0 + 8.0 * t1)) / t01_4 + (v1 * (8.0 * t0 + 7.0 * t1)) / t01_4;
	// 	c = (a0 * (t0_2 / 2 + 3.0 * t0 * t1 + (3.0 * t1_2) / 2.0)) / t01_3 + (p0 * (10.0 * t0_2 + 40.0 * t0 * t1 + 10.0 * t1_2)) / t01_5 - (p1 * (10.0 * t0_2 + 40.0 * t0 * t1 + 10.0 * t1_2)) / t01_5 - (v0 * (4.0 * t0_2 + 20.0 * t0 * t1 + 6.0 * t1_2)) / t01_4 - (v1 * ((3.0 * t0_2) / 2.0 + 3.0 * t0 * t1 + t1_2 / 2.0)) / t01_3 - (v1 * (6.0 * t0_2 + 20.0 * t0 * t1 + 4.0 * t1_2)) / t01_4;
	// 	d = (t0 * v1 * (t0_2 + 6.0 * t0 * t1 + 3.0 * t1_2)) / (2.0 * t01_3) - (a0 * t1 * (3.0 * t0_2 + 6.0 * t0 * t1 + t1_2)) / (2.0 * t01_3) - (30.0 * p0 * t0 * t1 * (t0 + t1)) / t01_5 + (30.0 * p1 * t0 * t1 * (t0 + t1)) / t01_5 + (6.0 * t0 * t1 * v0 * (2.0 * t0 + 3.0 * t1)) / t01_4 + (6.0 * t0 * t1 * v1 * (3.0 * t0 + 2.0 * t1)) / t01_4;
	// 	e = (30.0 * p0 * t0_2 * t1_2) / t01_5 - (30.0 * p1 * t0_2 * t1_2) / t01_5 - (t0_2 * v1 * (4.0 * t0 * t1 - t0_2 + 12 * t1_2)) / t01_4 - (t1_2 * v0 * (12.0 * t0_2 + 4.0 * t0 * t1 - t1_2)) / t01_4 + (a0 * t0 * t1_2 * (3.0 * t0 + 2.0 * t1)) / (2.0 * t01_3) - (t0_2 * t1 * v1 * (2.0 * t0 + 3.0 * t1)) / (2.0 * t01_3);
	// 	f = (p1 * t0_3 * (t0_2 - 5.0 * t0 * t1 + 10.0 * t1_2)) / t01_5 - (p0 * t1_3 * (10.0 * t0_2 - 5.0 * t0 * t1 + t1_2)) / t01_5 - (a0 * t0_2 * t1_3) / (2.0 * t01_3) + (t0_3 * t1_2 * v1) / (2.0 * t01_3) + (t0 * t1_3 * v0 * (4.0 * t0 - t1)) / t01_4 - (t0_3 * t1 * v1 * (t0 - 4.0 * t1)) / t01_4;
}

void JRS_Trajectory::SetTrapezoidVelAcc(double vel, double acc)
{
	SetTrapezoidVel(vel);
	SetTrapezoidAcc(acc);
}

void JRS_Trajectory::SetTrapezoidVel(double vel)
{
	m_fTrapezoidMaxVel = FABS(vel);
}

void JRS_Trajectory::SetTrapezoidAcc(double acc)
{
	m_fTrapezoidAcc = FABS(acc);
}

void JRS_Trajectory::SetTrapezoid(double fT0, double fT1, double fP0, double fP1, double fV0, double fV1, double fA0, double fA1)
{
	double delta_p;
	double max_vel_time;
	double acc_time;

	max_vel_time = (m_fTrapezoidMaxVel / m_fTrapezoidAcc);

	if( fP0 > fP1 )
		m_fTrapezoidActAcc = -m_fTrapezoidAcc;
	else
		m_fTrapezoidActAcc = m_fTrapezoidAcc;


	delta_p = FABS(fP1 - fP0);

	if( max_vel_time * m_fTrapezoidMaxVel > delta_p ) {
		// 가감속 시간 보다 이동 거리가 짧은 경우
		acc_time = SQRT(delta_p / m_fTrapezoidAcc);

		m_fTrapezoidT0 = fT0;
		m_fTrapezoidT1 = m_fTrapezoidT0 + acc_time;
		m_fTrapezoidT2 = m_fTrapezoidT1;
		m_fTrapezoidT3 = m_fTrapezoidT2 + acc_time;
		m_fTrapezoidActVelMax = m_fTrapezoidActAcc * acc_time;
	} else {
		// 최고 속도에서 동작 가능 구간
		m_fTrapezoidActVelMax = m_fTrapezoidActAcc * max_vel_time;
		m_fTrapezoidT0 = fT0;
		m_fTrapezoidT1 = m_fTrapezoidT0 + max_vel_time;
		m_fTrapezoidT2 = m_fTrapezoidT1 + ((delta_p - (max_vel_time * m_fTrapezoidMaxVel)) / m_fTrapezoidMaxVel);
		m_fTrapezoidT3 = m_fTrapezoidT2 + max_vel_time;

	}

	m_fTrapezoidT01 = m_fTrapezoidT1 - m_fTrapezoidT0;
	m_fTrapezoidT12 = m_fTrapezoidT2 - m_fTrapezoidT1;
	m_fTrapezoidT23 = m_fTrapezoidT3 - m_fTrapezoidT2;
}

void JRS_Trajectory::GetTrapezoid(double t, double* pos, double* vel, double* acc)
{
	double dt;
	double max_vel_time;
	max_vel_time = (m_fTrapezoidMaxVel / m_fTrapezoidAcc);

	if( t < m_fTrapezoidT0 ) {
		last_p = p0;
		last_v = v0;
		last_a = a0;
	} else if( t < m_fTrapezoidT1 ) {
		dt = t - m_fTrapezoidT0;
		last_a = m_fTrapezoidActAcc;
		last_v = m_fTrapezoidActAcc * dt;
		last_p = p0 + (last_v * dt / 2.0);
	} else if( t < m_fTrapezoidT2 && m_fTrapezoidT2 != m_fTrapezoidT1 ) {
		dt = t - m_fTrapezoidT1;
		last_a = 0.0;
		last_v = m_fTrapezoidT01 * m_fTrapezoidActAcc;
		last_p = p0 + ((m_fTrapezoidT01 * m_fTrapezoidActVelMax) / 2.0) + (m_fTrapezoidActVelMax * dt);
	} else if( t <= m_fTrapezoidT3 ) {
		dt = t - m_fTrapezoidT2;
		last_a = -m_fTrapezoidActAcc;
		last_v = m_fTrapezoidActVelMax - (m_fTrapezoidActAcc * dt);
		last_p = p0 + (m_fTrapezoidActVelMax * (m_fTrapezoidT01 + m_fTrapezoidT12)) - (((m_fTrapezoidT23 - dt) * last_v) / 2.0);
	} else {
		last_p = p1;
		last_v = v1;
		last_a = a1;
	}

	GetLastTrajectory(pos, vel, acc);
}

void JRS_Trajectory::SetLinear(double fT0, double fT1, double fP0, double fP1)
{
	m_fLinearDeltaT = fT1 - fT0;
	m_fLinearDeltaP = fP1 - fP0;
	m_fLinearConst = m_fLinearDeltaP / m_fLinearDeltaT;
}

void JRS_Trajectory::GetLinear(double t, double* pos, double* vel, double* acc)
{
	double fDT;

	if (t < t0)
		t = t0;
	else if (t > t1)
		t = t1;

	fDT = t - t0;

	last_p = p0 + (m_fLinearConst * fDT);
	last_v = 0.0;
	last_a = 0.0;

	GetLastTrajectory(pos, vel, acc);
}

void JRS_Trajectory::SetSine(double fMAG, double fFREQ, double fCYCLE)
{
	m_fSineMag		= fMAG/2.0;
	m_fSineFreq		= fFREQ;
	m_fSineCycle	= fCYCLE;

	m_fSineOmega	= 2.0 * _PI * m_fSineFreq;

	if (m_fSineFreq == 0)
	{
		m_fSineT = 0.0;
	}
	else
	{
		m_fSineT = 1.0 / m_fSineFreq;
	}
	
	if (m_fSineCycle < 0.0)
	{
		m_fSineCycle = 999999999.0;
	}
	m_fSineTf = m_fSineT * m_fSineCycle;
}

void JRS_Trajectory::GetSine(double t, double* pos, double* vel, double* acc)
{
	if (t < t0)
		t = t0;
	else if (t > m_fSineTf)
		t = m_fSineTf;

	*pos = m_fSineMag*(1.0 - cos_(m_fSineOmega * t)) + p0;
	*vel = (m_fSineMag * m_fSineOmega) * sin_(m_fSineOmega * t);
	*acc = (m_fSineMag * m_fSineOmega * m_fSineOmega) * cos_(m_fSineOmega * t);
}