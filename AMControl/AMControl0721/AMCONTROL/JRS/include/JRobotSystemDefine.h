#pragma once

#ifdef TC_BUILD
#define USE_TWINCAT3
#endif

#define JRS_DYNAMIC_SIZE	
#ifndef JRS_DYNAMIC_SIZE
#define JRS_MAX_DOF		7
#endif

enum JRS_ERR {
	ERR_NONE = 0,
	ERR_DOF,
	ERR_TRAJ_TYPE,
	ERR_TRAJ_TRAPAZODIAL_ACC,
	ERR_JCTRL_MODE,
	ERR_JCTRL_MISS,
};

#define JRS_CTRL_MODE_JOINT		0
#define JRS_CTRL_MODE_TASK		1

#define JRS_TRAJ_5TH			0
#define JRS_TRAJ_TRAPEZOID		1
#define JRS_TRAJ_LINEAR			2
#define JRS_TRAJ_SIN			3

#define JRS_JCTRL_MODE_POS		0
#define JRS_JCTRL_MODE_VEL		1
#define JRS_JCTRL_MODE_TORQ_POS	2
#define JRS_JCTRL_MODE_TORQ_VEL	3
#define JRS_JCTRL_MODE_TORQ		4

#define JRS_JCTRL_MODE_PID		0
#define JRS_JCTRL_MODE_PD		1
#define JRS_JCTRL_MODE_PI		2

#define JRS_JCTRL_NONE		0xFFFFFFFFFFFFFFFF


#ifndef _PI	
#define _PI	3.141592653589793238462643383279502884197169399375105
#endif

#ifndef _R2D
#define _R2D	(180.0 / _PI)
#endif

#ifndef _D2R
#define _D2R	(_PI / 180.0)
#endif

#ifndef _CycleTime
#define _CycleTime	0.001
#define DT	_CycleTime
#endif

#ifndef NULL
#define NULL	0
#endif

#ifdef USE_TWINCAT3
#define COS(x)		cos_(x)
#define SIN(x)		sin_(x)
#define ATAN2(x, y) atan2_(x, y)
#define FABS(x)		fabs_(x)
#define SQRT(x)		sqrt_(x)
#define POW(x, y)	pow_(x, y)
#define ACOS(x)		acos_(x)
#else
#include <math.h>

#define COS(x)		cos(x)
#define SIN(x)		sin(x)
#define ATAN2(x, y)	atan2(x, y)
#define FABS(x)		fabs(x)
#define SQRT(x)		sqrt(x)
#define POW(x, y)	pow(x, y)
#define ACOS(x)		acos(x)
#endif