#include "../include/JRobotSystemDefine.h"

#ifdef USE_TWINCAT3
#include "TcPch.h"
#pragma hdrstop
#else
#include <memory.h>
#endif

#include "MRobotControl.h"
//#include <memory.h>


#ifndef NULL
#define NULL		0
#endif

int		m_nDoF;
DH *	m_pDH;
double *	q;
double *	qdot;
double *	qddot;

TaskPoint	P;
TaskPoint	Pdot;
TaskPoint	Pddot;

TaskPoint	Tool;

double *	torq;


MRobotControl::MRobotControl(void)
	: m_nDoF(0)
	, m_pDH(NULL)
	, m_q(NULL)
	, m_qdot(NULL)
	, m_qddot(NULL)
	, m_torq(NULL)
	, m_Grav(NULL)
	, dt(0.001)
	, m_pKineWorld(NULL)
	, m_pKineLocal(NULL)
	, m_vpL(NULL)
{
}

MRobotControl::~MRobotControl(void)
{
	ClearController();
	LoadDH(NULL, false);
}

int MRobotControl::LoadDH(char * szFileName, bool bDynmics)
{
	//FILE * fp;
	//fopen_s(&fp, szFileName, "r");

	//if( fp == NULL ) 
	//	return MR_FILE_ERROR;

	ClearController();

	//fscanf_s(fp, "%d", &m_nDoF);
	m_nDoF = 7;

	m_pDH	= new DH[m_nDoF];
	m_q		= new double[m_nDoF];
	m_qdot	= new double[m_nDoF];
	m_qddot	= new double[m_nDoF];
	m_torq	= new double[m_nDoF];
	m_Grav = new double[3];

	m_pKineWorld = new KineInfo[m_nDoF];
	m_pKineLocal = new KineInfo[m_nDoF];
	m_vpL = new JMatrix[m_nDoF];

	m_JacobP.SetMatrix(3, m_nDoF);
	m_Jacob.SetMatrix(6, m_nDoF);
	m_JacobPdot.SetMatrix(3, m_nDoF);
	m_Jacobdot.SetMatrix(6, m_nDoF);
	m_qdot_prev.SetMatrix(m_nDoF,1);

	memset(m_q, 0, sizeof(double) * m_nDoF);
	memset(m_qdot, 0, sizeof(double) * m_nDoF);
	memset(m_qddot, 0, sizeof(double) * m_nDoF);
	memset(m_torq, 0, sizeof(double) * m_nDoF);

	for (int i = 0; i < m_nDoF; i++)
	{
		m_vpL[i].SetMatrix(3, 1);
		m_vpL[i].setZero();
	}

	m_JacobP.setZero();
	m_Jacob.setZero();
	m_JacobPdot.setZero();
	m_Jacobdot.setZero();
	m_qdot_prev.setZero();

	m_Grav[0] = 0.0;
	m_Grav[1] = 0.0;
	m_Grav[2] = -9.81;

	//m_pKineWorld[0].Pos;

	//for (int i = 0; i < m_nDoF; i++)
	//	m_q[i] = 3.141592 / 10;

	m_pDH[0].a = 0.0;
	m_pDH[1].a = 0.05;
	m_pDH[2].a = 0.6;
	m_pDH[3].a = 0.02;
	m_pDH[4].a = 0.0;
	m_pDH[5].a = 0.0;
	m_pDH[6].a = 0.0;

	m_pDH[0].alpha = 3.141592/2;
	m_pDH[1].alpha = -3.141592 / 2;
	m_pDH[2].alpha = 0.0;
	m_pDH[3].alpha = -3.141592 / 2;
	m_pDH[4].alpha = 3.141592 / 2;
	m_pDH[5].alpha = -3.141592 / 2;
	m_pDH[6].alpha = 0.0;

	m_pDH[0].d = 0.0;
	m_pDH[1].d = 0.383;
	m_pDH[2].d = 0.0;
	m_pDH[3].d = 0.0;
	m_pDH[4].d = 0.6;
	m_pDH[5].d = 0.0;
	m_pDH[6].d = 0.083+0.15;

	m_pDH[0].theta = 0.0;
	m_pDH[1].theta = 0.0;
	m_pDH[2].theta = -3.141592 / 2;
	m_pDH[3].theta = 0.0;
	m_pDH[4].theta = 0.0;
	m_pDH[5].theta = 0.0;
	m_pDH[6].theta = 0.0;

	m_pDH[0].m = 1.0;
	m_pDH[1].m = 1.0;
	m_pDH[2].m = 1.0;
	m_pDH[3].m = 1.0;
	m_pDH[4].m = 1.0;
	m_pDH[5].m = 1.0;
	m_pDH[6].m = 1.0;

	m_pDH[0].I.setIdentity();
	m_pDH[1].I.setIdentity();
	m_pDH[2].I.setIdentity();
	m_pDH[3].I.setIdentity();
	m_pDH[4].I.setIdentity();
	m_pDH[5].I.setIdentity();
	m_pDH[6].I.setIdentity();

	m_pDH[0].r.setZero();
	m_pDH[1].r.setZero();
	m_pDH[2].r.setZero();
	m_pDH[3].r.setZero();
	m_pDH[4].r.setZero();
	m_pDH[5].r.setZero();
	m_pDH[6].r.setZero();

	m_pKineWorld0.Pos.setZero();
	//m_pKineWorld0.R.setIdentity();
	m_pKineWorld0.R(0, 0) = 1.0;	m_pKineWorld0.R(0, 1) = 0.0;	m_pKineWorld0.R(0, 2) = 0.0;
	m_pKineWorld0.R(1, 0) = 0.0;	m_pKineWorld0.R(1, 1) = cos_(-_PI / 2);	m_pKineWorld0.R(1, 2) = -sin_(-_PI / 2);
	m_pKineWorld0.R(2, 0) = 0.0;	m_pKineWorld0.R(2, 1) = sin_(-_PI / 2);	m_pKineWorld0.R(2, 2) = cos_(-_PI / 2);

	m_pKineLocal0.Pos = m_pKineWorld0.Pos;
	m_pKineLocal0.R = m_pKineWorld0.R;

	m_pDH[0].j_type = 1;
	m_pDH[1].j_type = 0;
	m_pDH[2].j_type = 0;
	m_pDH[3].j_type = 0;
	m_pDH[4].j_type = 0;
	m_pDH[5].j_type = 0;
	m_pDH[6].j_type = 0;

	m_vq.SetMatrix(m_nDoF, 1);
	m_vqdot.SetMatrix(m_nDoF, 1);
	m_vqddot.SetMatrix(m_nDoF, 1);
	m_vq.setZero();
	m_vqdot.setZero();
	m_vqddot.setZero();

	Create_Var_Kine(m_nDoF);

	if(bDynmics)
	{
		Create_Var_Dyn(m_nDoF);
		SetGravityMode();
	}

	//for(int i = 0; i < m_nDoF; i++ ) {
	//	fscanf_s(fp, "%lf", &m_pDH[i].a);
	//	fscanf_s(fp, "%lf", &m_pDH[i].alpha);
	//	fscanf_s(fp, "%lf", &m_pDH[i].d);
	//	fscanf_s(fp, "%lf", &m_pDH[i].theta);

	//	if( bDynmics ) {
	//		fscanf_s(fp, "%lf", &m_pDH[i].m);

	//		//fscanf_s(fp, "%lf", &m_pDH[i].r[0]);
	//		//fscanf_s(fp, "%lf", &m_pDH[i].r[1]);
	//		//fscanf_s(fp, "%lf", &m_pDH[i].r[2]);
	//		//for(int j = 0; j < 3; j++ )
	//		//	for(int k = 0; k < 3; k++ ) 
	//		//		fscanf_s(fp, "%lf", &m_pDH[i].I[j][k]);

	//		fscanf_s(fp, "%lf", &m_pDH[i].r(0, 0));
	//		fscanf_s(fp, "%lf", &m_pDH[i].r(1, 0));
	//		fscanf_s(fp, "%lf", &m_pDH[i].r(2, 0));
	//		for (int j = 0; j < 3; j++)
	//			for (int k = 0; k < 3; k++)
	//				fscanf_s(fp, "%lf", &m_pDH[i].I(j, k));
	//	}

	//	fscanf_s(fp, "%d", &m_pDH[i].nType);
	//}
	//fclose(fp);

	return MR_NO_ERR;
}

int MRobotControl::Init(double q[], TaskPoint & sBase)
{
 	if( m_nDoF <= 0 )
 		return MR_NO_DH;
 	
	TaskPoint	Base;

 	memcpy(m_q, q, sizeof(double) * m_nDoF);
	for (int i = 0 ; i < m_nDoF; i++)
	{
		m_qdot[i] = 0.0;
		m_qddot[i] = 0.0;
	}
 	m_qdot_prev.setZero();

	Base = sBase;
 	//Base.Map2Vec();
 	m_pKineWorld0.Pos = Base.P;
 	////double Rx;
 	////double Ry;
 	////double Rz;
 	////sBase.GetEuler(Rx, Ry, Rz);
 	sBase.GetQ2RMat(m_pKineWorld0.R);
// 	m_pKineLocal0 = m_pKineWorld0;	

	return MR_NO_ERR;
}

void MRobotControl::SetDH(int idx, double a, double alpha, double d, double theta)
{
	m_pDH[idx].a = a;
	m_pDH[idx].alpha = alpha;
	m_pDH[idx].d = d;
	m_pDH[idx].theta = theta;
}

void MRobotControl::SetTool(TaskPoint & sTool)
{
	m_Tool = sTool;
}

//TaskPoint MRobotControl::ForwardKinematics(double q[], bool bUpdate)
//{
//	// uUpdate 가 true 이면 내부 변수에 forward 결과 및 조인트 값 업데이트, false 면 입력받은 joint 값으로 forward 푼 최종 결과만 리턴
//	TaskPoint sFW;
//
//	return sFW;
//}


void MRobotControl::ClearController(void)
{
	const TaskPoint sZero;

	m_nDoF = 0;

	if( m_pDH )
		delete [] m_pDH;
	m_pDH = NULL;

	if( m_q )
		delete [] m_q;
	m_q = NULL;

	if(m_qdot )
		delete [] m_qdot;
	m_qdot = NULL;

	if(m_qddot )
		delete [] m_qddot;
	m_qddot = NULL;

	if ( m_Grav )
		delete[] m_Grav;
	m_Grav = NULL;

	//P = Pdot = Pddot = Base = Tool = sZero;

	if( torq )
		delete [] torq;
	torq = NULL;

	if (m_pKineWorld)
		delete[] m_pKineWorld;
	m_pKineWorld = NULL;

	if (m_pKineLocal)
		delete[] m_pKineLocal;
	m_pKineLocal = NULL;

	if (m_vpL)
		delete[] m_vpL;
	m_vpL = NULL;
}

int MRobotControl::CTM_Cont(double q[], double qdot[], double qddot[], double *q_torq)
{
	for (int i = 0 ; i < m_nDoF; i++)
	{
		m_vq(i,0) = q[i];
		m_vqdot(i,0) = qdot[i];
		m_vqddot(i,0) = qddot[i];
	}
	return 0;
}
int MRobotControl::Comp_Cont(double q[], double FT_sens[], double *q_torq)
{
	return 0;
}
int MRobotControl::Comp_Int(double q[], double qdot[], double qddot[], double *q_torq)
{
	return 0;
}