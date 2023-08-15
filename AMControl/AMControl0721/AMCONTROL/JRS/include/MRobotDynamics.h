#pragma once
#include "mrobotkinematics.h"
class MRobotDynamics :
	public MRobotKinematics
{
	JMatrix m_vInvTorq;
	JMatrix m_vInvG;

	KineInfo* m_pTempKineWorld;
	KineInfo* m_pTempKineLocal;
	JMatrix* m_vpForce;
	JMatrix* m_vpTorque;
	JMatrix m_vGrav;

	JMatrix m_vq_dyn;
	JMatrix m_vqdot_dyn;
	JMatrix m_vqddot_dyn;

	JMatrix m_M;
	JMatrix m_C;
	JMatrix m_G;

public:
	MRobotDynamics(void);
	~MRobotDynamics(void);

public:
	virtual	int Create_Var_Dyn(int DoF);
	virtual int SetGravityMode(void);

	virtual int InverseDynamics(double * q_torq);
	virtual int InverseDynamics(double q[], double qdot[], double qddot[], double * q_torq);

	virtual	JMatrix	InverseDynamic(void);
	virtual	JMatrix	InverseDynamic(KineInfo* TempLocal, KineInfo* TempWorld);
	virtual	JMatrix	InverseDynamic_G(KineInfo* TempLocal, KineInfo* TempWorld);

	virtual	JMatrix GetM(const JMatrix& q);
	virtual	JMatrix GetC(const JMatrix& q, const JMatrix& qdot);
	virtual	JMatrix GetC(void);
	virtual	JMatrix GetG(const JMatrix& q);
	virtual	JMatrix GetG(void);
};

