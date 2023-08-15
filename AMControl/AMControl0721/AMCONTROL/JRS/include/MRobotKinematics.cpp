#include "../include/JRobotSystemDefine.h"

#ifdef USE_TWINCAT3
#include "TcPch.h"
#pragma hdrstop
#else
#include <memory.h>
#endif

#include "MRobotKinematics.h"
//#include <memory.h>

MRobotKinematics::MRobotKinematics(void)
	: MRobotControl()
	, Z0(3, 1)
	, H(4,4)
	, FowardH(4,4)
	, ret_transformation(4, 4)
	, ret_rotx(4, 4)
	, ret_roty(4, 4)
	, ret_rotz(4, 4)
	, ret_transx(4, 4)
	, ret_transy(4, 4)
	, ret_transz(4, 4)
	, m_Temp33Mat(3, 3)
	, m_TempVec6(6, 1)
	, m_TempVec3(3, 1)
	, Err(6, 1)
	, ErrDot(6, 1)
	, Dot(6, 1)
	, DDot(6, 1)
	, Kp_gain(6, 6)
	, E(6, 6)
	, Rd(3, 3)
{
	Z0.setZero();
	Z0(2, 0) = 1.0;
	m_Temp33Mat.IsIdentity();

	H.setIdentity();
	FowardH.setIdentity();
	
	ret_transformation.setIdentity();
	ret_rotx.setIdentity();
	ret_roty.setIdentity();
	ret_rotz.setIdentity();
	ret_transx.setIdentity();
	ret_transy.setIdentity();
	ret_transz.setIdentity();

	m_TempVec3.setZero();
	m_TempVec6.setZero();
	m_Temp33Mat.setZero();

	Err.setZero();
	ErrDot.setZero();
	Dot.setZero();
	DDot.setZero();

	Kp_gain.setIdentity();
	Kp_gain.SetTopLeftCorner(Kp_gain.GetTopLeftCorner(3, 3) * 2000);
	Kp_gain.SetBottomRightCorner(Kp_gain.GetBottomRightCorner(3, 3) * 100);
	//Kp_gain.setZero();
	
	E.setIdentity();
	E = E * 0.0001;

	Rd.setIdentity();
}


MRobotKinematics::~MRobotKinematics(void)
{
}

int MRobotKinematics::Create_Var_Kine(int DoF)
{
	q_vec.SetMatrix(DoF, 1);
	qdot_vec.SetMatrix(DoF, 1);
	qddot_vec.SetMatrix(DoF, 1);
	for (int i = 0; i < m_nDoF; i++)
		q_vec(i, 0) = m_q[i];

	qdot_vec.setZero();
	qddot_vec.setZero();

	W.SetMatrix(DoF, DoF);
	W.setIdentity();
	W.setIdentity();
	W(0, 0) = 0.05;

	return 0;
}

int MRobotKinematics::InverseKinematics(TaskPoint P, TaskPoint Pdot, double * q_ref)
{
	if( m_nDoF <= 0 )
		return MR_NO_DH;

	InverseKinematic(P, Pdot);
	memcpy(q_ref, m_q, sizeof(double) * m_nDoF);
	//q_ref = m_q;

	return MR_NO_ERR;
}

int MRobotKinematics::InverseKinematics(TaskPoint P, TaskPoint Pdot, TaskPoint Pddot, double * q_ref)
{
	if( m_nDoF <= 0 )
		return MR_NO_DH;

	InverseKinematic(P, Pdot, Pddot);

	memcpy(q_ref, m_q, sizeof(double) * m_nDoF);

	return MR_NO_ERR;
}

//void MRobotKinematics::InverseKinematic(TaskPoint& Task, TaskPoint& Taskdot)
//{
//
//	JMatrix Err(3, 1);
//
//	double kp = 1000;
//	double kd = 1;
//
//	JMatrix E(3, 3);
//	//////////////////////////////////////////////////////////////////////////
//	m_JacobP.setZero();
//	m_JacobPdot.setZero();
//	Err.setZero();
//
//	JMatrix q_vec(m_nDoF, 1);
//	JMatrix qdot_vec(m_nDoF, 1);
//	JMatrix qddot_vec(m_nDoF, 1);
//	for (int i = 0; i < m_nDoF; i++)
//	{
//		q_vec(i, 0) = m_q[i];
//		qdot_vec(i, 0) = m_qdot[i];
//	}
//	qddot_vec.setZero();
//
//	ComputeForward(q_vec, qdot_vec);
//
//	GetJacobianP();
//	GetJacobianPdot();
//	//////////////////////////////////////////////////////////////////////////
//	E.setIdentity();
//	E *= 0.000001;
//	//////////////////////////////////////////////////////////////////////////
//	Err = Task.P + m_pKineWorld[m_nDoF - 1].Pos * -1;
//	//////////////////////////////////////////////////////////////////////////
//
//	if (m_nDoF > 3)
//		qdot_vec.SetTopRows(m_JacobP.Transpose() * (m_JacobP * m_JacobP.Transpose() + E).GetInverse() * (Taskdot.P + kp * Err));
//	else if (m_nDoF == 3)
//		qdot_vec.SetTopRows(m_JacobP.Transpose() * (m_JacobP * m_JacobP.Transpose() + E).GetInverse() * (Taskdot.P + kp * Err));
//	else if (m_nDoF == 2)
//		qdot_vec.SetTopRows(m_JacobP.GetTopRows(2).Transpose() * (m_JacobP.GetTopRows(2) * m_JacobP.GetTopRows(2).Transpose() + E.GetTopLeftCorner(2, 2)).GetInverse() * (Taskdot.P.GetTopRows(2) + kp * Err.GetTopRows(2)));
//
//	q_vec = q_vec + qdot_vec * dt;
//	qddot_vec = (qdot_vec + m_qdot_prev * -1) / dt;
//
//	m_qdot_prev = qdot_vec;
//
//	for (int i = 0; i < m_nDoF; i++)
//	{
//		m_q[i] = q_vec(i, 0);
//		m_qdot[i] = qdot_vec(i, 0);
//		m_qddot[i] = qddot_vec(i, 0);
//	}
//}

void MRobotKinematics::InverseKinematic(TaskPoint& Task, TaskPoint& Taskdot)
{
	//return;
	//////////////////////////////////////////////////////////////////////////

	Err.setZero();
	ErrDot.setZero();
	Dot.setZero();
	DDot.setZero();
	m_Jacob.setZero();
	m_Jacobdot.setZero();

	for (int i = 0; i < m_nDoF; i++)
	{
		q_vec(i, 0) = m_q[i];
		qdot_vec(i, 0) = m_qdot[i];
	}
	qddot_vec.setZero();

	ComputeForward(q_vec, qdot_vec);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//JQuaternion quat_err;
	//////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////////
	GetJacobian();
	//////////////////////////////////////////////////////////////////////////
	Dot.SetTopRows(Taskdot.P);
	Dot.SetBottomRows(Task.Rd*Taskdot.O);
	//////////////////////////////////////////////////////////////////////////
	////quat_err = JQuaternion(TrajQuart.Traj_R.Transpose() * Curr_R);
	double Rx;
	double Ry;
	double Rz;
	
	Task.GetEuler(Rx, Ry, Rz);
	////Rd = Task.Rd*(RotX(Rx)*RotY(Ry)*RotZ(Rz)).GetTopLeftCorner(3,3);
	Task.GetQ2RMat(m_Temp33Mat);
	Rd = Task.Rd* m_Temp33Mat;
	m_Temp33Mat = Rd.GetTranspose() * m_pKineWorld[m_nDoF - 1].R;

	m_Temp33Mat.SetCol(0, m_Temp33Mat.GetCol(0) / m_Temp33Mat.GetCol(0).norm());
	m_Temp33Mat.SetCol(1, m_Temp33Mat.GetCol(1) + m_Temp33Mat.GetCol(0) * (m_Temp33Mat.GetCol(0).GetTranspose() * m_Temp33Mat.GetCol(1)) * -1);
	m_Temp33Mat.SetCol(2, m_Temp33Mat.GetCol(0).cross(m_Temp33Mat.GetCol(1)));
	m_Temp33Mat.SetCol(2, m_Temp33Mat.GetCol(2) / m_Temp33Mat.GetCol(2).norm());

	quat_err.SetEuler(m_Temp33Mat);
	////Rd.GetTranspose()* m_pKineWorld[m_nDoF - 1].R;
	//quat_err.SetEuler(Rd.GetTranspose()*m_pKineWorld[m_nDoF - 1].R);
	//quat_err.O.setZero();

	////////////////////////////////////////////////////////////////////////////
	Err.SetTopRows(Task.P + m_pKineWorld[m_nDoF-1].Pos * -1);
	Err.SetBottomRows( Rd*quat_err.O*-1);
	////////////////////////////////////////////////////////////////////////////

	if (m_nDoF > 6){
		qdot_vec = (W * W.GetTranspose())*m_Jacob.GetTranspose() * ((m_Jacob* (W * W.GetTranspose()) * m_Jacob.GetTranspose() + E).GetInverse()) * (Dot + Kp_gain * Err);// +1000 * TrajC.weight * MakeNULLMotion(m_Tqd.q, KMode, NMode, Jacobian);
	}
	else {
		qdot_vec = (W*W.GetTranspose())*m_Jacob.GetTranspose()*((m_Jacob*(W*W.GetTranspose())*m_Jacob.GetTranspose() + E).GetInverse())*(Dot + Kp_gain * Err);
	}
	//m_Tqd.qddot = (m_Tqd.qdot - Prev_qdot) / dt + Jacobian.transpose()*(Jacobian*Jacobian.transpose() + E).inverse()*(kp*Err + kd*ErrDot);
	
	q_vec = q_vec + qdot_vec * dt;
	qddot_vec = (qdot_vec + m_qdot_prev * -1) / dt;
	m_qdot_prev = qdot_vec;

	for (int i = 0; i < m_nDoF; i++)
	{
		m_q[i] = q_vec(i, 0);
		m_qdot[i] = qdot_vec(i, 0);
		m_qddot[i] = qddot_vec(i, 0);
	}
}

void MRobotKinematics::InverseKinematic(TaskPoint& Task, TaskPoint& Taskdot, TaskPoint& Taskddot)
{
	//return;
	//////////////////////////////////////////////////////////////////////////
	//Task.Map2Vec();
	//Taskdot.Map2Vec();
	//Taskddot.Map2Vec();

	m_JacobP.setZero();
	m_JacobPdot.setZero();
	Err.setZero();
	ErrDot.setZero();

	//H = ComputeForward(m_Tqd.q);								//modified youngsu 20180321
	qddot_vec.setZero();
	for (int i = 0; i < m_nDoF; i++)
	{
		q_vec(i, 0) = m_q[i];
		qdot_vec(i, 0) = m_qdot[i];
	}

	ComputeForward(q_vec, qdot_vec);
	//Curr_Pos = H.GetCol(3).GetTopRows(3);

	m_JacobP = GetJacobianP();
	m_JacobPdot = GetJacobianPdot();
	//////////////////////////////////////////////////////////////////////////
	Err.SetTopRows(Task.P + m_pKineWorld[m_nDoF - 1].Pos * -1);
	ErrDot.SetTopRows(Taskdot.P + m_JacobP * qdot_vec * -1);
	//////////////////////////////////////////////////////////////////////////

	if (m_nDoF > 3)
		qdot_vec = m_JacobP.GetTranspose() * (m_JacobP * m_JacobP.GetTranspose() + E.GetTopLeftCorner(3,3)).GetInverse() * (Taskdot.P + Kp_gain.GetTopLeftCorner(3,3) * Err.GetTopRows(3));
	else if (m_nDoF == 3)
		qdot_vec = m_JacobP.GetTranspose() * (m_JacobP * m_JacobP.GetTranspose() + E.GetTopLeftCorner(3, 3)).GetInverse() * (Taskdot.P + Kp_gain.GetTopLeftCorner(3, 3) * Err.GetTopRows(3));
	else if (m_nDoF == 2)
		qdot_vec = m_JacobP.GetTopRows(2).GetTranspose() * (m_JacobP.GetTopRows(2) * m_JacobP.GetTopRows(2).GetTranspose() + E.GetTopLeftCorner(2, 2)).GetInverse() * (Taskdot.P.GetTopRows(2) + Kp_gain.GetTopLeftCorner(2,2) * Err.GetTopRows(2));

	if (m_nDoF > 3)
		qddot_vec = m_JacobP.GetTranspose() * (m_JacobP * m_JacobP.GetTranspose() + E.GetTopLeftCorner(3,3)).GetInverse() * (Taskddot.P + Kp_gain.GetTopLeftCorner(3,3) * ErrDot.GetTopRows(3) + m_JacobPdot * qdot_vec * -1); //m_Tqd.qddot = (m_Tqd.qdot - Prev_qdot) / dt + m_JacobP.Transpose()*(m_JacobP*m_JacobP.Transpose() + E).GetInverse()*(kp*Err + kd*ErrDot);
	else if (m_nDoF == 3)
		qddot_vec = m_JacobP.GetTranspose() * (m_JacobP * m_JacobP.GetTranspose() + E.GetTopLeftCorner(3,3)).GetInverse() * (Taskddot.P + Kp_gain.GetTopLeftCorner(3,3) * ErrDot.GetTopRows(3) + m_JacobPdot * qdot_vec * -1);
	else if (m_nDoF == 2)
		qddot_vec = m_JacobP.GetTopRows(2).GetTranspose() * (m_JacobP.GetTopRows(2) * m_JacobP.GetTopRows(2).GetTranspose() + E.GetTopLeftCorner(2, 2)).GetInverse() * (Taskddot.P.GetTopRows(2) + Kp_gain.GetTopLeftCorner(2,2) * ErrDot.GetTopRows(2) + m_JacobPdot.GetTopRows(2) * qdot_vec * -1);

	q_vec = q_vec + qdot_vec * dt;

	for (int i = 0; i < m_nDoF; i++)
	{
		m_q[i] = q_vec(i, 0);
		m_qdot[i] = qdot_vec(i, 0);
		m_qddot[i] = qddot_vec(i, 0);
	}
}

//int MRobotKinematics::SetGravityMode(void)
//{
//	// 아무것도 구현안함
//	return MR_NO_DYNAMIC;
//}

//int	  MRobotKinematics::InverseDynamics(double * q_torq)
//{
//	// 아무것도 구현안함
//	return MR_NO_DYNAMIC;
//}
//
//int MRobotKinematics::InverseDynamics(double q[], double qdot[], double qddot[], double * q_torq)
//{
//	return MR_NO_DYNAMIC;
//}

void	MRobotKinematics::Set_Init_qd(const double* q)
{
	//return;
	for (int i = 0; i < m_nDoF; i++)
	{
		m_q[i] = q[i];
		m_qdot[i] = 0.0;
		m_qddot[i] = 0.0;
	}
}

//void	MRobotKinematics::Set_HT0(JMatrix H)
//{
//	m_BaseH = H;
//}

void MRobotKinematics::ForwardKinematics(TaskPoint& sFW, double q[], bool bUpdate)
{
	//return;
	// uUpdate 가 true 이면 내부 변수에 forward 결과 및 조인트 값 업데이트, false 면 입력받은 joint 값으로 forward 푼 최종 결과만 리턴
	q_vec.setZero();
	for (int i = 0; i < m_nDoF; i++)
		q_vec(i,0) = q[i];
	
 	ComputeForward(q_vec);
	//if (bUpdate == false)
	//	return sFW;
	
	//sFW.Set_Pd(m_pKineWorld[m_nDoF - 1].Pos);
	sFW.P = m_pKineWorld[m_nDoF - 1].Pos;
	sFW.SetEuler(m_pKineWorld[m_nDoF - 1].R);
	//sFW.Set_Rd(m_pKineWorld[m_nDoF - 1].R);
	sFW.Rd = m_pKineWorld[m_nDoF - 1].R;

	//sFW.P(0, 0) = 100.0;
	//sFW.P(1, 0) = 200.0;
	//sFW.P(2, 0) = 300.0;
	//m_Temp33Mat.setIdentity();
	//sFW.SetEuler(m_Temp33Mat);
	//sFW.Rd = m_Temp33Mat;
}

void MRobotKinematics::ComputeForward(const JMatrix& q)
{
	//return;
	ComputeForward(q, m_pKineLocal, m_pKineWorld);
}
void 	MRobotKinematics::ComputeForward(const JMatrix& q, const JMatrix& qdot)
{
	//return;
	ComputeForward(q, qdot, m_pKineLocal, m_pKineWorld);
}
void	MRobotKinematics::ComputeForward(const JMatrix& q, const JMatrix& qdot, const JMatrix& qddot)
{
	//return;
	ComputeForward(q, qdot, qddot, m_pKineLocal, m_pKineWorld);
}

void MRobotKinematics::ComputeForward(const JMatrix& q, KineInfo* TempInLocal, KineInfo* TempInWorld)
{
	//return;
	FowardH.setIdentity();
 	FowardH.SetTopLeftCorner(m_pKineWorld0.R);
 	FowardH.SetTopRightCorner(m_pKineWorld0.Pos);

	int i = 0;
	do {
  		H = Transformation(i, q(i, 0));
  		// Compute joint position vector in world coordinate
  		FowardH = FowardH * H;
 
 		TempInWorld[i].Pos = FowardH.GetTopRightCorner(3, 1);
		TempInWorld[i].z = FowardH.GetCol(2).GetTopRows(3);
		TempInWorld[i].R = FowardH.GetTopLeftCorner(3, 3);

		TempInLocal[i].Pos = H.GetTopRightCorner(3, 1);
		TempInLocal[i].z = H.GetCol(2).GetTopRows(3);
		TempInLocal[i].R = H.GetTopLeftCorner(3, 3);
 
 		m_vpL[i] = H.GetTopLeftCorner(3, 3).Transpose() * TempInLocal[i].Pos;
 
 	} while (++i < m_nDoF);

	//return FowardH;
}
void	MRobotKinematics::ComputeForward(const JMatrix& q, const JMatrix& qdot, const JMatrix& qddot, KineInfo* TempInLocal, KineInfo* TempInWorld)
{
	//return;
	ComputeForward(q, qdot, TempInLocal, TempInWorld);
	int i = 1;

	if (m_pDH[i].j_type == 0){
		TempInLocal[0].OmegaDot = TempInLocal[0].R.Transpose() * (Z0 * qddot(0, 0) + m_pKineLocal0.OmegaDot + m_pKineLocal0.Omega.cross(Z0) * qdot(i, 0));
		TempInLocal[0].Acc = TempInLocal[0].R.Transpose() * m_pKineLocal0.Acc + TempInLocal[0].OmegaDot.cross(m_vpL[0]) + TempInLocal[0].Omega.cross(TempInLocal[0].Omega.cross(m_vpL[0]));
		TempInLocal[0].Acc_c = TempInLocal[0].Acc + TempInLocal[0].OmegaDot.cross(m_pDH[0].r * -1) + TempInLocal[0].Omega.cross(TempInLocal[0].Omega.cross(m_pDH[0].r * -1));
		// Compute joint acceleration in world coordinate
		TempInWorld[0].Acc = m_pKineWorld0.R * TempInLocal[0].Acc;
	}
	else if(m_pDH[i].j_type == 1){
		TempInLocal[0].OmegaDot = TempInLocal[0].R.Transpose() * m_pKineLocal0.OmegaDot;
		TempInLocal[0].Acc = TempInLocal[0].R.Transpose() * (m_pKineLocal0.Acc + Z0 * qddot(0,0)) + TempInLocal[0].OmegaDot.cross(m_vpL[0]) + TempInLocal[0].Omega.cross(TempInLocal[0].Omega.cross(m_vpL[0])) + TempInLocal[0].Omega.cross(TempInWorld[0].R.GetTranspose()*(TempInWorld[0].Vel - m_pKineWorld0.Vel));
		TempInLocal[0].Acc_c = TempInLocal[0].Acc + TempInLocal[0].OmegaDot.cross(m_pDH[0].r * -1) + TempInLocal[0].Omega.cross(TempInLocal[0].Omega.cross(m_pDH[0].r * -1));
		// Compute joint acceleration in world coordinate
		TempInWorld[0].Acc = m_pKineWorld0.R * TempInLocal[0].Acc;
	}

	do {
		if (m_pDH[i].j_type == 0){
			// Compute joint acceleration in local coordinate
			TempInLocal[i].OmegaDot = TempInLocal[i].R.Transpose() * (Z0 * qddot(i, 0) + TempInLocal[i - 1].OmegaDot + TempInLocal[i - 1].Omega.cross(Z0) * qdot(i, 0));
			TempInLocal[i].Acc = TempInLocal[i].R.Transpose() * TempInLocal[i - 1].Acc + TempInLocal[i].OmegaDot.cross(m_vpL[i]) + TempInLocal[i].Omega.cross(TempInLocal[i].Omega.cross(m_vpL[i]));
			TempInLocal[i].Acc_c = TempInLocal[i].Acc + TempInLocal[i].OmegaDot.cross(m_pDH[i].r * -1) + TempInLocal[i].Omega.cross(TempInLocal[i].Omega.cross(m_pDH[i].r * -1));
			// Compute joint acceleration in world coordinate
			TempInWorld[i].Acc = TempInWorld[i - 1].R * TempInLocal[i].Acc;
		}
		else if (m_pDH[i].j_type == 1){
			// Compute joint acceleration in local coordinate
			TempInLocal[i].OmegaDot = TempInLocal[i].R.Transpose() * TempInLocal[i - 1].OmegaDot;
			TempInLocal[i].Acc = TempInLocal[i].R.Transpose() * (TempInLocal[i - 1].Acc + Z0 * qddot(i,0)) + TempInLocal[i].OmegaDot.cross(m_vpL[i]) + TempInLocal[i].Omega.cross(TempInLocal[i].Omega.cross(m_vpL[i])) + TempInLocal[i].Omega.cross(TempInWorld[i].R.GetTranspose()*(TempInWorld[i].Vel - TempInWorld[i-1].Vel));
			TempInLocal[i].Acc_c = TempInLocal[i].Acc + TempInLocal[i].OmegaDot.cross(m_pDH[i].r * -1) + TempInLocal[i].Omega.cross(TempInLocal[i].Omega.cross(m_pDH[i].r * -1));
			// Compute joint acceleration in world coordinate
			TempInWorld[i].Acc = TempInWorld[i - 1].R * TempInLocal[i].Acc;
		}
		
	} while (++i < m_nDoF);
}
void	MRobotKinematics::ComputeForward(const JMatrix& q, const JMatrix& qdot, KineInfo* TempInLocal, KineInfo* TempInWorld)
{
	//return;
	ComputeForward(q, TempInLocal, TempInWorld);

	int i = 1;
	// Compute joint velocity
	if (m_pDH[i].j_type == 0){
		TempInLocal[0].Omega = TempInLocal[0].R.Transpose() * (m_pKineLocal0.Omega + Z0 * qdot(0, 0));
		TempInWorld[0].Omega = m_pKineWorld0.Omega + m_pKineWorld0.R * Z0 * qdot(0, 0);
		TempInWorld[0].Vel = m_pKineWorld0.Vel + TempInWorld[0].Omega.cross(TempInWorld[0].Pos - m_pKineWorld0.Pos);
	}
	else if(m_pDH[i].j_type == 1){
		TempInLocal[0].Omega = TempInLocal[0].R.Transpose() * m_pKineLocal0.Omega;
		TempInWorld[0].Omega = m_pKineWorld0.Omega;
		TempInWorld[0].Vel = m_pKineWorld0.Vel + m_pKineWorld0.R * Z0 * qdot(0,0) + TempInWorld[0].Omega.cross(TempInWorld[0].Pos - m_pKineWorld0.Pos);
	}
	

	do {
		// Compute joint velocity in local coordinate
		if (m_pDH[i].j_type == 0){
			TempInLocal[i].Omega = TempInLocal[i].R.Transpose() * (TempInLocal[i - 1].Omega + Z0 * qdot(i, 0));
			TempInWorld[i].Omega = TempInWorld[i - 1].R.GetCol(2) * qdot(i, 0) + TempInWorld[i - 1].Omega;
			TempInWorld[i].Vel = TempInWorld[i].Omega.cross(TempInWorld[i].Pos - TempInWorld[i - 1].Pos) + TempInWorld[i - 1].Vel;
		}
		else if(m_pDH[i].j_type == 1){
			TempInLocal[i].Omega = TempInLocal[i].R.Transpose() * TempInLocal[i - 1].Omega;
			TempInWorld[i].Omega = TempInWorld[i - 1].Omega;
			TempInWorld[i].Vel = TempInWorld[i].Omega.cross(TempInWorld[i].Pos - TempInWorld[i - 1].Pos) + TempInWorld[i - 1].Vel + TempInWorld[i - 1].R.GetCol(2) * qdot(i, 0);
		}

	} while (++i < m_nDoF);
}
void	MRobotKinematics::ComputeForward_qddot(const JMatrix& qddot, KineInfo* TempInLocal, KineInfo* TempInWorld)
{
	//return;
	int i = 1;

	TempInLocal[0].OmegaDot = TempInLocal[0].R.Transpose() * (Z0 * qddot(0, 0) + m_pKineLocal0.OmegaDot + m_pKineLocal0.Omega.cross(Z0) * 0.0);
	TempInLocal[0].Acc = TempInLocal[0].R.Transpose() * m_pKineLocal0.Acc + TempInLocal[0].OmegaDot.cross(m_vpL[0]) + TempInLocal[0].Omega.cross(TempInLocal[0].Omega.cross(m_vpL[0]));
	TempInLocal[0].Acc_c = TempInLocal[0].Acc + TempInLocal[0].OmegaDot.cross(m_pDH[0].r * -1) + TempInLocal[0].Omega.cross(TempInLocal[0].Omega.cross(m_pDH[0].r * -1));
	// Compute joint acceleration in world coordinate
	TempInWorld[0].Acc = m_pKineWorld0.R * TempInLocal[0].Acc;

	do {
		// Compute joint acceleration in local coordinate
		TempInLocal[i].OmegaDot = TempInLocal[i].R.Transpose() * (Z0 * qddot(i, 0) + TempInLocal[i - 1].OmegaDot + TempInLocal[i - 1].Omega.cross(Z0) * 0.0);
		TempInLocal[i].Acc = TempInLocal[i].R.Transpose() * TempInLocal[i - 1].Acc + TempInLocal[i].OmegaDot.cross(m_vpL[i]) + TempInLocal[i].Omega.cross(TempInLocal[i].Omega.cross(m_vpL[i]));
		TempInLocal[i].Acc_c = TempInLocal[i].Acc + TempInLocal[i].OmegaDot.cross(m_pDH[i].r * -1) + TempInLocal[i].Omega.cross(TempInLocal[i].Omega.cross(m_pDH[i].r * -1));
		// Compute joint acceleration in world coordinate
		TempInWorld[i].Acc = TempInWorld[i - 1].R * TempInLocal[i].Acc;
	} while (++i < m_nDoF);
}

JMatrix MRobotKinematics::Transformation(const int nIdx, const double q)
{
	
	ret_transformation.setIdentity();

 	if (m_pDH[nIdx].j_type == 0) 
 		ret_transformation = RotZ(q+m_pDH[nIdx].theta)*TransZ(m_pDH[nIdx].d)*RotX(m_pDH[nIdx].alpha)*TransX(m_pDH[nIdx].a);
 	else if (m_pDH[nIdx].j_type == 1)
 		ret_transformation =  RotZ(m_pDH[nIdx].theta)*TransZ(q+m_pDH[nIdx].d)*RotX(m_pDH[nIdx].alpha)*TransX(m_pDH[nIdx].a);

	return ret_transformation;
}

// Make homogeneous matrix about alpha(x-axis) rotation
JMatrix & MRobotKinematics::RotX(const double theta)
{
	
	ret_rotx.setZero();
	ret_rotx(0, 0) = 1.0;
	ret_rotx(1, 1) = COS(theta);
	ret_rotx(1, 2) = -SIN(theta);
	ret_rotx(2, 1) = SIN(theta);
	ret_rotx(2, 2) = COS(theta);
	ret_rotx(3, 3) = 1.0;

	return ret_rotx;
}
JMatrix & MRobotKinematics::RotY(const double theta)
{
	ret_roty.setZero();
	ret_roty(0, 0) = COS(theta);
	ret_roty(0, 2) = SIN(theta);
	ret_roty(1, 1) = 1.0;
	ret_roty(2, 0) = -SIN(theta);
	ret_roty(2, 2) = COS(theta);
	ret_roty(3, 3) = 1.0;

	return ret_roty;
}
// Make homogeneous matrix about theta(z-axis) rotation
JMatrix & MRobotKinematics::RotZ(const double theta)
{
	ret_rotz.setZero();
	ret_rotz(0, 0) = COS(theta);
	ret_rotz(0, 1) = -SIN(theta);
	ret_rotz(1, 0) = SIN(theta);
	ret_rotz(1, 1) = COS(theta);
	ret_rotz(2, 2) = 1.0;
	ret_rotz(3, 3) = 1.0;

	return ret_rotz;
}

JMatrix & MRobotKinematics::TransX(const double x)
{
	ret_transx.setIdentity();
	ret_transx(0, 3) = x;

	return ret_transx;
}

JMatrix & MRobotKinematics::TransY(const double y)
{
	ret_transy.setIdentity();
	ret_transy(1, 3) = y;

	return ret_transy;
}

JMatrix & MRobotKinematics::TransZ(const double z)
{
	ret_transz.setIdentity();
	ret_transz(2, 3) = z;

	return ret_transz;
}

JMatrix& MRobotKinematics::GetJacobianP(KineInfo* TempWorld)
{
	int i;
	m_JacobP.setZero();

	for (i = 0; i < m_nDoF; i++) {
		if (i == 0)
		{
			if (m_pDH[i].j_type == 0)
			{
				m_JacobP.SetCol(i, m_pKineWorld0.R.GetCol(2).cross(TempWorld[m_nDoF - 1].Pos + m_pKineWorld0.Pos * -1));
			}
			else if(m_pDH[i].j_type == 1)
			{
				m_JacobP.SetCol(i, m_pKineWorld0.R.GetCol(2));
			}
		}
		else
		{
			if (m_pDH[i].j_type == 0){
				m_JacobP.SetCol(i, TempWorld[i - 1].R.GetCol(2).cross(TempWorld[m_nDoF - 1].Pos + TempWorld[i - 1].Pos * -1));
			}
			else if(m_pDH[i].j_type	 == 1){
				m_JacobP.SetCol(i, TempWorld[i - 1].R.GetCol(2));
			}
		}
	}
	return m_JacobP;
}
JMatrix& MRobotKinematics::GetJacobian(KineInfo* TempWorld)
{
	int i;

	m_Jacob.setZero();
	m_TempVec6.setZero();

	for (i = 0; i < m_nDoF; i++) {
		if (i == 0) {
			if (m_pDH[i].j_type == 0){
				m_TempVec6.SetTopRows(m_pKineWorld0.R.GetCol(2).cross(TempWorld[m_nDoF - 1].Pos + m_pKineWorld0.Pos * -1));
				m_TempVec6.SetBottomRows(m_pKineWorld0.R.GetCol(2));
			}
			else if (m_pDH[i].j_type == 1){
				m_TempVec6.setZero();
				m_TempVec6.SetTopRows(m_pKineWorld0.R.GetCol(2));
			}
			m_Jacob.SetCol(i, m_TempVec6);
		}
		else {
			if (m_pDH[i].j_type == 0){
				m_TempVec6.SetTopRows(TempWorld[i - 1].R.GetCol(2).cross(TempWorld[m_nDoF - 1].Pos + TempWorld[i - 1].Pos * -1));
				m_TempVec6.SetBottomRows(TempWorld[i - 1].R.GetCol(2));
			}
			else if (m_pDH[i].j_type == 1){
				m_TempVec6.setZero();
				m_TempVec6.SetTopRows(TempWorld[i - 1].R.GetCol(2));
			}
			m_Jacob.SetCol(i, m_TempVec6);
		}
	}
	return m_Jacob;
}
JMatrix& MRobotKinematics::GetJacobianP(const JMatrix& q)
{
	ComputeForward(q, m_pKineLocal, m_pKineWorld);
	return GetJacobianP(m_pKineWorld);
}
JMatrix& MRobotKinematics::GetJacobian(const JMatrix& q)
{
	ComputeForward(q, m_pKineLocal, m_pKineWorld);
	return GetJacobian(m_pKineWorld);
}
JMatrix& MRobotKinematics::GetJacobian(void)
{
	return GetJacobian(m_pKineWorld);
}
JMatrix& MRobotKinematics::GetJacobianP(void)
{
	return GetJacobianP(m_pKineWorld);
}

JMatrix& MRobotKinematics::GetJacobianPdot(KineInfo* TempWorld)
{
	int i;
	//JMatrix	Jacob_dot(3, m_nDoF);

	m_JacobPdot.setZero();

	for (i = 0; i < m_nDoF; i++) {
		if (i == 0) {
			if (m_pDH[i].j_type == 0){
				m_JacobPdot.SetCol(i, m_pKineWorld0.R.GetCol(2).cross(TempWorld[m_nDoF - 1].Vel - m_pKineWorld0.Vel) + (TempWorld[i].Omega.cross(m_pKineWorld0.R.GetCol(2))).cross(TempWorld[m_nDoF - 1].Pos + m_pKineWorld0.Pos * -1));
			}
			else if (m_pDH[i].j_type == 1){
				//m_JacobPdot.SetCol(i, m_pKineWorld0.R.GetCol(2).cross(TempWorld[m_nDoF - 1].Vel - m_pKineWorld0.Vel) + (TempWorld[i].Omega.cross(m_pKineWorld0.R.GetCol(2))).cross(TempWorld[m_nDoF - 1].Pos + m_pKineWorld0.Pos * -1));
			}
		}
		else {
			if (m_pDH[i].j_type == 0){
				m_JacobPdot.SetCol(i, TempWorld[i - 1].R.GetCol(2).cross(TempWorld[m_nDoF - 1].Vel + TempWorld[i - 1].Vel * -1) + (TempWorld[i].Omega.cross(TempWorld[i - 1].R.GetCol(2))).cross(TempWorld[m_nDoF - 1].Pos + TempWorld[i - 1].Pos * -1));
			}
			else if (m_pDH[i].j_type == 1){
				//m_JacobPdot.SetCol(i, TempWorld[i - 1].R.GetCol(2).cross(TempWorld[m_nDoF - 1].Vel + TempWorld[i - 1].Vel * -1) + (TempWorld[i].Omega.cross(TempWorld[i - 1].R.GetCol(2))).cross(TempWorld[m_nDoF - 1].Pos + TempWorld[i - 1].Pos * -1));
			}
		}
	}
	return m_JacobPdot;
}
JMatrix& MRobotKinematics::GetJacobiandot(KineInfo* TempWorld)
{
	int i;

	m_Jacobdot.setZero();
	m_TempVec6.setZero();

	for (i = 0; i < m_nDoF; i++) {
		if (i == 0) {
			if (m_pDH[i].j_type == 0){
				m_TempVec6.SetTopRows(m_pKineWorld0.R.GetCol(2).cross(TempWorld[m_nDoF - 1].Vel - m_pKineWorld0.Vel) + (TempWorld[i].Omega.cross(m_pKineWorld0.R.GetCol(2))).cross(TempWorld[m_nDoF - 1].Pos + m_pKineWorld0.Pos * -1));
				m_TempVec6.SetBottomRows(TempWorld[i].Omega.cross(m_pKineWorld0.R.GetCol(2)));
			}
			else if (m_pDH[i].j_type == 1)
			{
				//m_TempVec6.SetTopRows(Z0.cross(TempWorld[m_nDoF - 1].Vel) + (TempWorld[i].Omega.cross(Z0)).cross(TempWorld[m_nDoF - 1].Pos + P0 * -1));
				//m_TempVec6.SetBottomRows(TempWorld[i].Omega.cross(Z0));
			}
			m_Jacobdot.SetCol(i, m_TempVec6);
		}
		else {
			if (m_pDH[i].j_type == 0){
				m_TempVec6.SetTopRows(TempWorld[i - 1].R.GetCol(2).cross(TempWorld[m_nDoF - 1].Vel + TempWorld[i - 1].Vel * -1) + (TempWorld[i].Omega.cross(TempWorld[i - 1].R.GetCol(2))).cross(TempWorld[m_nDoF - 1].Pos + TempWorld[i - 1].Pos * -1));
				m_TempVec6.SetBottomRows(TempWorld[i].Omega.cross(TempWorld[i - 1].R.GetCol(2)));
			}
			else if (m_pDH[i].j_type == 1){
				//m_TempVec6.SetTopRows(TempWorld[i - 1].R.GetCol(2).cross(TempWorld[m_nDoF - 1].Vel + TempWorld[i - 1].Vel * -1) + (TempWorld[i].Omega.cross(TempWorld[i - 1].R.GetCol(2))).cross(TempWorld[m_nDoF - 1].Pos + TempWorld[i - 1].Pos * -1));
				//m_TempVec6.SetBottomRows(TempWorld[i].Omega.cross(TempWorld[i - 1].R.GetCol(2)));
			}
			m_Jacobdot.SetCol(i, m_TempVec6);
		}
	}
	return m_Jacobdot;
}
JMatrix& MRobotKinematics::GetJacobianPdot(const JMatrix& q)
{
	ComputeForward(q, m_pKineLocal, m_pKineWorld);
	return GetJacobianPdot(m_pKineWorld);
}
JMatrix& MRobotKinematics::GetJacobiandot(const JMatrix& q)
{
	ComputeForward(q, m_pKineLocal, m_pKineWorld);
	return GetJacobiandot(m_pKineWorld);
}
JMatrix& MRobotKinematics::GetJacobianPdot(void)
{
	return GetJacobianPdot(m_pKineWorld);
}
JMatrix& MRobotKinematics::GetJacobiandot(void)
{
	return GetJacobiandot(m_pKineWorld);
}