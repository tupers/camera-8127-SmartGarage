#ifndef GVSS_H_
#define GVSS_H_

#include "gvssProtocol.h"
#include <modbus.h>

#define PLCSERVER_PORT				8900

#define MAX_MATRIX_NUM		50
#define MAX_ACT_NUM			40
#define GVSSTOPLC_FLAG		254
#define GVSSTOPLC_SIZE		400

typedef struct _FinalResult
{
	unsigned int position;
	unsigned int distance;
}FinalResult;

typedef struct 
{
	unsigned short flag[4];
	unsigned short size;
	unsigned short data[MAX_MATRIX_NUM*3+2];
}GvsstoPlc;

typedef struct _GvssObj
{
	Gvss_Protocol Com;
	modbus_t *ctx;
	unsigned short *pData;
}GvssObj;

#endif
