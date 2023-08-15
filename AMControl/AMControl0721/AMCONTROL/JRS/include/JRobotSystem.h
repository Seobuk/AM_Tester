#pragma once

#include "JRobotSystemDefine.h"
#include "JRS_MDStateControl.h"

#ifdef JRS_JOINT_ONLY
#include "../JointControl/JRS_JointControl.h"
#endif

#include "../PathPlan/JRS_Trajectory.h"
#include "../include/MRobotControl.h"
#include "../include/MRobotDynamics.h"
#include "../include/MRobotKinematics.h"