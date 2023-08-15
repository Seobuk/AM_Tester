///////////////////////////////////////////////////////////////////////////////
// AMCONTROLDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "AMCONTROLDriver.h"
#include "AMCONTROLClassFactory.h"

DECLARE_GENERIC_DEVICE(AMCONTROLDRV)

IOSTATUS CAMCONTROLDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CAMCONTROLClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CAMCONTROLDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CAMCONTROLDriver::AMCONTROLDRV_GetVersion( )
{
	return( (AMCONTROLDRV_Major << 8) | AMCONTROLDRV_Minor );
}

