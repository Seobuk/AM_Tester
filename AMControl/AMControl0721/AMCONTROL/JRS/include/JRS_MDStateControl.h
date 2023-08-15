#pragma once

typedef unsigned short UNSIGNED16;


#define	SERVO_OFF		0
#define	SERVO_ON		1
#define	FAULT_RESET		2


enum MD_STATE {
	ELMO_STATE_ERROR		=	-1,
	NOT_READY_TO_SWITCH_ON	=	0,
	SWITCH_ON_DISABLE		=	1,
	READY_TO_SWITCH_ON		=	2,
	SWITCHED_ON				=	3,
	OPERATION_ENABLED		=	4,
	QUICK_STOP_ACTIVE		=	5,
	FAULT_REACTION_ACTGIVE	=	6,
	FAULT					=	7
};

class JRS_MDStateControl
{
	UNSIGNED16 *	m_pStatusWord;
	UNSIGNED16 		m_xControlWord;

	long			m_eCommand;

	UNSIGNED16		m_xStatusWordSingleMode;

public:
	JRS_MDStateControl(void);
	JRS_MDStateControl(UNSIGNED16 * pStatusWord);
	~JRS_MDStateControl(void);

public:
	UNSIGNED16	GetStatusWord(void);
	UNSIGNED16	GetControlWord(long eCommand, UNSIGNED16 xStatusWord);

	MD_STATE	GetStatus(void);

	bool		IsServo(void);

private:
	void		CommandServoON(void);
	void		CommandServerOFF(void);
	void		CommandFaultReset(void);
};

