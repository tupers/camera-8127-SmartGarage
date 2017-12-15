#ifndef GVSS_CLIENT_H_
#define GVSS_CLIENT_H_

#include "gvssProtocol.h"

#define CLIENTPORT					9800


typedef struct _GvssObj
{
	Gvss_Protocol Com;
	char *pData;
}GvssClientObj;

#endif
