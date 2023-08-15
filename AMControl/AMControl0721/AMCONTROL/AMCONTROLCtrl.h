///////////////////////////////////////////////////////////////////////////////
// AMCONTROLCtrl.h

#ifndef __AMCONTROLCTRL_H__
#define __AMCONTROLCTRL_H__

#include <atlbase.h>
#include <atlcom.h>


#include "resource.h"       // main symbols
#include "AMCONTROLW32.h"
#include "TcBase.h"
#include "AMCONTROLClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CAMCONTROLCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CAMCONTROLCtrl, &CLSID_AMCONTROLCtrl>
	, public IAMCONTROLCtrl
	, public ITcOCFCtrlImpl<CAMCONTROLCtrl, CAMCONTROLClassFactory>
{
public:
	CAMCONTROLCtrl();
	virtual ~CAMCONTROLCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_AMCONTROLCTRL)
DECLARE_NOT_AGGREGATABLE(CAMCONTROLCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CAMCONTROLCtrl)
	COM_INTERFACE_ENTRY(IAMCONTROLCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __AMCONTROLCTRL_H__
