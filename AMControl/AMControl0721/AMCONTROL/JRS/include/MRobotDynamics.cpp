#include "../include/JRobotSystemDefine.h"

#ifdef USE_TWINCAT3
#include "TcPch.h"
#pragma hdrstop
#else
#include <memory.h>
#endif

#include "MRobotDynamics.h"
//#include <memory.h>

MRobotDynamics::MRobotDynamics(void)
	: MRobotKinematics()
	, m_vpForce(NULL)
	, m_vpTorque(NULL)
	, m_pTempKineWorld(NULL)
	, m_pTempKineLocal(NULL)
{
}


MRobotDynamics::~MRobotDynamics(void)
{
	if (m_vpForce)
		delete[] m_vpForce;
	m_vpForce = NULL;

	if (m_vpTorque)
		delete[] m_vpTorque;
	m_vpTorque = NULL;

	if (m_pTempKineWorld)
		delete[] m_pTempKineWorld;
	m_pTempKineWorld = NULL;

	if (m_pTempKineLocal)
		delete[] m_pTempKineLocal;
	m_pTempKineLocal = NULL;
}

int MRobotDynamics::Create_Var_Dyn(int DoF)
{
	m_vInvTorq.SetMatrix(DoF, 1);
	m_vInvG.SetMatrix(DoF, 1);
	m_vGrav.SetMatrix(3, 1);
	m_vq_dyn.SetMatrix(DoF, 1);
	m_vqdot_dyn.SetMatrix(DoF, 1);
	m_vqddot_dyn.SetMatrix(DoF, 1);
	m_M.SetMatrix(DoF, DoF);
	m_C.SetMatrix(DoF, 1);
	m_G.SetMatrix(DoF, 1);

	m_vInvTorq.setZero();
	m_vInvG.setZero();

	m_vq_dyn.setZero();
	m_vqdot_dyn.setZero();
	m_vqddot_dyn.setZero();

	m_M.setZero();
	m_C.setZero();
	m_G.setZero();

	m_vpForce = new JMatrix[DoF];
	m_vpTorque = new JMatrix[DoF];
	m_pTempKineWorld = new KineInfo[DoF];
	m_pTempKineLocal = new KineInfo[DoF];

	for (int i = 0; i < DoF; i++)
	{
		m_vpForce[i].SetMatrix(3, 1);
		m_vpTorque[i].SetMatrix(3, 1);
		m_vpForce[i].setZero();
		m_vpTorque[i].setZero();
	}
	for (int i = 0; i < 3; i++)
		m_vGrav(i, 0) = m_Grav[i];
	return 0;
}

int MRobotDynamics::SetGravityMode(void)
{
	if( m_nDoF <= 0 )
		return MR_NO_DH;

	for (int i = 0; i < 3; i++)
		m_vGrav(i, 0) = m_Grav[i];

	return MR_NO_ERR;
}

int MRobotDynamics::InverseDynamics(double * q_torq)
{
	if( m_nDoF <= 0 )
		return MR_NO_DH;

	int ret;
	ret = InverseDynamics(m_q, m_qdot, m_qddot, q_torq);
	memcpy(m_torq, q_torq, sizeof(double) * m_nDoF);

	return ret;
}

int MRobotDynamics::InverseDynamics(double q[], double qdot[], double qddot[], double * q_torq)
{
	if( m_nDoF <= 0 )
		return MR_NO_DH;

	// 전달받은 q, qdot, qddot을 이용한 inverse dynamics
	for (int i = 0; i < m_nDoF; i++)
	{
		m_vq_dyn(i, 0) = q[i];
		m_vqdot_dyn(i, 0) = qdot[i];
		m_vqddot_dyn(i, 0) = qddot[i];
	}
	ComputeForward(m_vq_dyn, m_vqdot_dyn, m_vqddot_dyn);
	
	m_vInvTorq = InverseDynamic(m_pKineLocal, m_pKineWorld);

	for (int i = 0; i < m_nDoF; i++)
		q_torq[i] = m_vInvTorq(i, 0);

	return MR_NO_ERR;
}

JMatrix MRobotDynamics::InverseDynamic(void)
{
	return InverseDynamic(m_pKineLocal, m_pKineWorld);
}
JMatrix MRobotDynamics::InverseDynamic(KineInfo* TempLocal, KineInfo* TempWorld)
{
	int i = m_nDoF - 2;

	m_vpForce[m_nDoF - 1] = TempLocal[m_nDoF - 1].R * (TempLocal[m_nDoF - 1].Acc_c + TempWorld[m_nDoF - 1].R.Transpose() * m_vGrav * -1) * m_pDH[m_nDoF - 1].m;
	m_vpTorque[m_nDoF - 1] = m_vpForce[m_nDoF - 1].cross(TempLocal[m_nDoF - 1].R * (m_vpL[m_nDoF - 1] + m_pDH[m_nDoF - 1].r * -1)) * -1 + TempLocal[m_nDoF - 1].R * (m_pDH[m_nDoF - 1].I * TempLocal[m_nDoF - 1].OmegaDot + TempLocal[m_nDoF - 1].Omega.cross(m_pDH[m_nDoF - 1].I * TempLocal[m_nDoF - 1].Omega));
	m_vInvTorq(m_nDoF - 1) = m_vpTorque[m_nDoF - 1](2);
	do {
		m_vpForce[i] = TempLocal[i].R * (m_vpForce[i + 1] + (TempLocal[i].Acc_c + TempWorld[i].R.Transpose() * m_vGrav * -1) * m_pDH[i].m);
		m_vpTorque[i] = m_vpForce[i].cross(TempLocal[i].R * (m_vpL[i] + m_pDH[i].r * -1)) * -1 + TempLocal[i].R * (m_vpTorque[i + 1] + m_vpForce[i + 1].cross(m_pDH[i].r * -1) + m_pDH[i].I * TempLocal[i].OmegaDot + TempLocal[i].Omega.cross(m_pDH[i].I * TempLocal[i].Omega));
		m_vInvTorq(i) = m_vpTorque[i](2);
	} while (--i >= 0);

	return m_vInvTorq;
}

JMatrix MRobotDynamics::InverseDynamic_G(KineInfo* TempLocal, KineInfo* TempWorld)
{
	int i = m_nDoF - 2;

	m_vpForce[m_nDoF - 1] = TempLocal[m_nDoF - 1].R * (TempWorld[m_nDoF - 1].R.Transpose() * m_vGrav * -1) * m_pDH[m_nDoF - 1].m;
	m_vpTorque[m_nDoF - 1] = m_vpForce[m_nDoF - 1].cross(TempLocal[m_nDoF - 1].R * (m_vpL[m_nDoF - 1] + m_pDH[m_nDoF - 1].r * -1)) * -1;
	m_vInvG(m_nDoF - 1) = m_vpTorque[m_nDoF - 1](2);
	do {
		m_vpForce[i] = TempLocal[i].R * (m_vpForce[i + 1] + (TempWorld[i].R.Transpose() * m_vGrav * -1) * m_pDH[i].m);
		m_vpTorque[i] = m_vpForce[i].cross(TempLocal[i].R * (m_vpL[i] + m_pDH[i].r * -1)) * -1 + TempLocal[i].R * (m_vpTorque[i + 1] + m_vpForce[i + 1].cross(m_pDH[i].r * -1));
		m_vInvG(i) = m_vpTorque[i](2);
	} while (--i >= 0);

	return m_vInvG;
}

JMatrix MRobotDynamics::GetM(const JMatrix& q)
{
	JMatrix Temp_qddot(m_nDoF, 1);

	Temp_qddot.setZero();

	ComputeForward(q, Temp_qddot, Temp_qddot, m_pTempKineLocal, m_pTempKineWorld);
	m_G = GetG();

	for (int i = 0; i < m_nDoF; i++)
	{
		Temp_qddot.setZero();
		Temp_qddot(i, 0) = 1.0;
		ComputeForward_qddot(Temp_qddot, m_pTempKineLocal, m_pTempKineWorld);
		m_M.SetCol(i, InverseDynamic(m_pTempKineLocal, m_pTempKineWorld) + m_G * -1);
	}
	return m_M;
}

JMatrix MRobotDynamics::GetC(const JMatrix& q, const JMatrix& qdot)
{
	JMatrix Temp_qddot(m_nDoF, 1);
	Temp_qddot.setZero();
	ComputeForward(q, qdot, Temp_qddot, m_pTempKineLocal, m_pTempKineWorld);
	m_G = InverseDynamic_G(m_pTempKineLocal, m_pTempKineWorld);
	m_C = InverseDynamic(m_pTempKineLocal, m_pTempKineWorld) - m_G;

	return m_C;
}
JMatrix MRobotDynamics::GetC(void)
{
	m_G = InverseDynamic_G(m_pKineLocal, m_pKineWorld);
	m_C = InverseDynamic(m_pKineLocal, m_pKineWorld) - m_G;
	return m_C;
}
JMatrix MRobotDynamics::GetG(const JMatrix& q)
{
	ComputeForward(q, m_pTempKineLocal, m_pTempKineWorld);
	m_G = InverseDynamic_G(m_pTempKineLocal, m_pTempKineWorld);
	return m_G;
}
JMatrix MRobotDynamics::GetG(void)
{
	m_G = InverseDynamic_G(m_pKineLocal, m_pKineWorld);
	return m_G;
}