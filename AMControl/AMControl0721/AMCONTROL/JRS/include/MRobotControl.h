#pragma once

#include "MRobotEnv.h"

class MRobotControl
{
protected:
	int		m_nDoF;
	DH *	m_pDH;
	double *	m_q;
	double *	m_qdot;
	double *	m_qddot;

	TaskPoint	m_P;
	TaskPoint	m_Pdot;
	TaskPoint	m_Pddot;

	TaskPoint	m_Base;
	TaskPoint	m_Tool;

	double *	m_torq;

	double dt;
	double *	m_Grav;

	KineInfo* m_pKineWorld;
	KineInfo* m_pKineLocal;
	JMatrix* m_vpL;
	JMatrix	m_JacobP;
	JMatrix	m_Jacob;
	JMatrix	m_JacobPdot;
	JMatrix	m_Jacobdot;

	KineInfo m_pKineWorld0;
	KineInfo m_pKineLocal0;
	JMatrix m_qdot_prev;

	JMatrix m_vq;
	JMatrix m_vqdot;
	JMatrix m_vqddot;

public:
	MRobotControl(void);
	~MRobotControl(void);

	int LoadDH(char * szFileName, bool bDynmics = false);
	
	int Init(double q[], TaskPoint & sBase);
	void SetDH(int idx, double a, double alpha, double d, double theta);

	void SetTool(TaskPoint & sTool);

	virtual void ForwardKinematics(TaskPoint& sFW, double q[], bool bUpdate = true) = 0;
	//virtual TaskPoint ForwardKinematics(double q[], bool bUpdate);

private:
	void ClearController(void);

public:
	virtual int InverseKinematics(TaskPoint P, TaskPoint Pdot, double * q_ref) = 0;
	virtual int InverseKinematics(TaskPoint P, TaskPoint Pdot, TaskPoint Pddot, double * q_ref) = 0;

public:
	virtual	int Create_Var_Dyn(int DoF) = 0;
	virtual int Create_Var_Kine(int DoF) = 0;

	virtual int SetGravityMode(void) = 0;
	
	virtual int InverseDynamics(double * q_torq) = 0;
	virtual int InverseDynamics(double q[], double qdot[], double qddot[], double * q_torq) = 0;

	virtual	int CTM_Cont(double q[], double qdot[], double qddot[], double *q_torq);
	virtual	int Comp_Cont(double q[], double FT_sens[], double *q_torq);
	virtual	int Comp_Int(double q[], double qdot[], double qddot[], double *q_torq);
};

