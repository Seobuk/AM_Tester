#include "JRobotSystemDefine.h"

#include "TcPch.h"
#pragma hdrstop


#include "JRS_MDStateControl.h"

#define STATE_MASK1	0x004F
#define STATE_MASK2	0x006F

#define _STATUS_WORD	(*m_pStatusWord)

JRS_MDStateControl::JRS_MDStateControl(void)
{
	m_eCommand = FAULT_RESET;

	m_xStatusWordSingleMode = 0;
	m_xControlWord = 0;

	m_pStatusWord = &m_xStatusWordSingleMode;
}

JRS_MDStateControl::JRS_MDStateControl(UNSIGNED16 * pStatusWord)
{
	m_eCommand = FAULT_RESET;

	m_xStatusWordSingleMode = 0;
	m_xControlWord = 0;

	m_pStatusWord = pStatusWord;
}


JRS_MDStateControl::~JRS_MDStateControl(void)
{
}

UNSIGNED16	JRS_MDStateControl::GetStatusWord(void)
{
	return _STATUS_WORD;
}

UNSIGNED16	JRS_MDStateControl::GetControlWord(long eCommand, UNSIGNED16 xStatusWord)
{
	m_eCommand = eCommand;
	
	_STATUS_WORD = xStatusWord;
	
	switch( m_eCommand ) {
	case SERVO_ON:
		CommandServoON();
		break;
	case SERVO_OFF:
		CommandServerOFF();
		break;
	case FAULT_RESET:
		CommandFaultReset();
		break;
	}

	return m_xControlWord;
}

MD_STATE	JRS_MDStateControl::GetStatus(void)
{
	if( (_STATUS_WORD & STATE_MASK1) == 0x0000 )
		return NOT_READY_TO_SWITCH_ON;
	else if( (_STATUS_WORD & STATE_MASK1) == 0x0040 )
		return SWITCH_ON_DISABLE;
	else if( (_STATUS_WORD & STATE_MASK2) == 0x0021 )
		return READY_TO_SWITCH_ON;
	else if( (_STATUS_WORD & STATE_MASK2) == 0x0023 )
		return SWITCHED_ON;
	else if( (_STATUS_WORD & STATE_MASK2) == 0x0027 )
		return OPERATION_ENABLED;
	else if( (_STATUS_WORD & STATE_MASK2) == 0x0007 )
		return QUICK_STOP_ACTIVE;
	else if( (_STATUS_WORD & STATE_MASK1) == 0x000F )
		return FAULT_REACTION_ACTGIVE;
	else if( (_STATUS_WORD & STATE_MASK1) == 0x0008 )
		return FAULT;
	return ELMO_STATE_ERROR;
}

bool		JRS_MDStateControl::IsServo(void)
{
	if( GetStatus() == OPERATION_ENABLED )
		return true;
	return false;
}

void		JRS_MDStateControl::CommandServoON(void)
{
	switch( GetStatus() ) {
		case SWITCH_ON_DISABLE:
			m_xControlWord = 0x0006;
			break;
		case READY_TO_SWITCH_ON:
			m_xControlWord = 0x0007;
			break;
		case SWITCHED_ON:
			m_xControlWord = 0x000F;
			break;
// 		case OPERATION_ENABLED:
// 			break;
		default:
			break;
	}

}

void		JRS_MDStateControl::CommandServerOFF(void)
{
	switch( GetStatus() ) {
//	 	case SWITCH_ON_DISABLE:
// 			break;
		case READY_TO_SWITCH_ON:
			m_xControlWord = 0x0000;
			break;
		case SWITCHED_ON:
			m_xControlWord = 0x0006;
			break;
		case OPERATION_ENABLED:
			m_xControlWord = 0x0007;
			break;
		default:
			break;
	}
}
void		JRS_MDStateControl::CommandFaultReset(void)
{
	MD_STATE eState = GetStatus();
	if( eState == NOT_READY_TO_SWITCH_ON || eState == FAULT_REACTION_ACTGIVE || eState == FAULT ) {
		m_xControlWord = 0x0080;
	} 
}
