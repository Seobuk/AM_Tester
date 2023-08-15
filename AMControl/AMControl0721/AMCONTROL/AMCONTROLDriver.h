///////////////////////////////////////////////////////////////////////////////
// AMCONTROLDriver.h

#ifndef __AMCONTROLDRIVER_H__
#define __AMCONTROLDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define AMCONTROLDRV_NAME        "AMCONTROL"
#define AMCONTROLDRV_Major       1
#define AMCONTROLDRV_Minor       0

#define DEVICE_CLASS CAMCONTROLDriver

#include "ObjDriver.h"

class CAMCONTROLDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl AMCONTROLDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(AMCONTROLDRV)
	VxD_Service( AMCONTROLDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __AMCONTROLDRIVER_H__