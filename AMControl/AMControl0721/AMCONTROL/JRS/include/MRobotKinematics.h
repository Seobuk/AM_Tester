#pragma once
#include "mrobotcontrol.h"
//#include "JMath.h"

class MRobotKinematics :
	public MRobotControl
{
public:
	JMatrix Z0;
	JMatrix	m_qdot_prev;
	JMatrix m_Temp33Mat;
	JMatrix m_TempVec6;
	JMatrix m_TempVec3;
	JMatrix H;
	JMatrix FowardH;
	JMatrix ret_transformation;
	JMatrix ret_rotx;
	JMatrix ret_roty;
	JMatrix ret_rotz;
	JMatrix ret_transx;
	JMatrix ret_transy;
	JMatrix ret_transz;

	JMatrix q_vec;
	JMatrix qdot_vec;
	JMatrix qddot_vec;

	JMatrix Err;
	JMatrix ErrDot;
	JMatrix Dot;
	JMatrix DDot;

	JMatrix Kp_gain;
	JMatrix W;
	JMatrix E;

	JMatrix Rd;
	TaskPoint quat_err;

	//JMatrix m_BaseH;

public:
	MRobotKinematics(void);
	~MRobotKinematics(void);

public:
	virtual int InverseKinematics(TaskPoint P, TaskPoint Pdot, double * q_ref);
	virtual int InverseKinematics(TaskPoint P, TaskPoint Pdot, TaskPoint Pddot, double * q_ref);

	virtual void ForwardKinematics(TaskPoint& sFW, double q[], bool bUpdate);

	virtual	void	InverseKinematic(TaskPoint& Task, TaskPoint& Taskdot);
	virtual	void	InverseKinematic(TaskPoint& Task, TaskPoint& Taskdot, TaskPoint& Taskddot);
	//TrajectoryQ &	InverseKinematic(TrajectoryC & TrajC, TrajectoryQuart & TrajQuart);

public:
	virtual int SetGravityMode(void) = 0;
	virtual	int Create_Var_Dyn(int DoF) = 0;
	virtual int Create_Var_Kine(int DoF);



	virtual int InverseDynamics(double * q_torq) = 0;
	virtual int InverseDynamics(double q[], double qdot[], double qddot[], double * q_torq) = 0;
	
	virtual	void	Set_Init_qd(const double * q);
	//virtual void	Set_HT0(JMatrix H);

	virtual	void ComputeForward(const JMatrix& q);
	virtual	void 	ComputeForward(const JMatrix& q, const JMatrix& qdot);
	virtual	void	ComputeForward(const JMatrix& q, const JMatrix& qdot, const JMatrix& qddot);
	virtual	void ComputeForward(const JMatrix& q, KineInfo* TempInLocal, KineInfo* TempInWorld);
	virtual	void	ComputeForward(const JMatrix& q, const JMatrix& qdot, KineInfo* TempInLocal, KineInfo* TempInWorld);
	virtual	void	ComputeForward(const JMatrix& q, const JMatrix& qdot, const JMatrix& qddot, KineInfo* TempInLocal, KineInfo* TempInWorld);
	virtual	void	ComputeForward_qddot(const JMatrix& qddot, KineInfo* TempInLocal, KineInfo* TempInWorld);

	virtual	JMatrix Transformation(const int nIdx, const double q);
	virtual	JMatrix & RotX(const double theta);
	virtual	JMatrix & RotY(const double theta);
	virtual	JMatrix & RotZ(const double theta);
	virtual	JMatrix & TransX(const double x);
	virtual	JMatrix & TransY(const double y);
	virtual	JMatrix & TransZ(const double z);

	virtual	JMatrix& GetJacobian(KineInfo* TempWorld);
	virtual	JMatrix& GetJacobianP(KineInfo* TempWorld);
	virtual	JMatrix& GetJacobian(const JMatrix& q);
	virtual	JMatrix& GetJacobianP(const JMatrix& q);
	virtual	JMatrix& GetJacobian(void);
	virtual	JMatrix& GetJacobianP(void);
	virtual	JMatrix& GetJacobianPdot(KineInfo* TempWorld);
	virtual	JMatrix& GetJacobiandot(KineInfo* TempWorld);
	virtual	JMatrix& GetJacobiandot(const JMatrix& q);
	virtual	JMatrix& GetJacobianPdot(const JMatrix& q);
	virtual	JMatrix& GetJacobiandot(void);
	virtual	JMatrix& GetJacobianPdot(void);
};

