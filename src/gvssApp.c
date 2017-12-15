#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "modebus_client.h"
#include "gvssApp.h"
#include "main.h"
#include "gvssProtocol.h"
#include "remote_debug_msg_drv.h"
#include "log.h"
#include "gvss_type.h"	

static GvssObj gGvss;
const uint16_t UT_REGISTERS_ADDRESS = 0x64;
/* Raise a manual exception when this adress is used for the first byte */
const uint16_t UT_REGISTERS_ADDRESS_SPECIAL = 0x65;
const uint16_t UT_REGISTERS_NB = 0x3;
uint16_t UT_REGISTERS_TAB[] = { 0x022B, 0x0001, 0x0064 };

static int InitGvss(GvssObj *pObj)
{		
	pObj->ctx = modbus_new_tcp(SMARTGARAGE_SLAVEIP, 502);
	if(pObj->ctx == NULL) 
	{
		fprintf(stderr, "Unable to allocate libmodbus context\n");
		return -1;
	}
	modbus_set_slave(pObj->ctx, SMARTGARAGE_SLAVEID);
	modbus_set_debug(pObj->ctx, FALSE);
	modbus_set_error_recovery(pObj->ctx,
					MODBUS_ERROR_RECOVERY_LINK |
					MODBUS_ERROR_RECOVERY_PROTOCOL);			
	return 0;
}

static void ExitGvss(GvssObj *pObj)
{
	modbus_free(pObj->ctx);
}


static unsigned int gGvssQuit = 0;
void Gvss_signalHandler(int signum)
{
    gGvssQuit = 1;

	printf("catch signal\n");
}

int gvss_dataFormat(Gvss_Result_Str *pResult,GvsstoPlc *info)
{
	unsigned int index;
	unsigned int t = 0;
	unsigned int size;
	
	Gvss_Result_BlockStr *recInfo;
	FinalResult *pFinalResult = (FinalResult *)((unsigned int)info + 320);
	
	info->flag[0] = GVSSTOPLC_FLAG;
	info->flag[1] = GVSSTOPLC_FLAG;
	info->flag[2] = GVSSTOPLC_FLAG;
	info->flag[3] = GVSSTOPLC_FLAG;
	
	pFinalResult->position = pResult->position;
	pFinalResult->distance = pResult->distance;

	info->size = sizeof(unsigned short)*2 + pResult->blockNum*sizeof(unsigned short)*3;
	size = sizeof(info->flag) + info->size + sizeof(info->size);
	
	info->data[t++] = pResult->blockNum;
	recInfo = pResult->block;
	for(index=0;index<pResult->blockNum;index++)
	{
		info->data[t++] = recInfo->area;
		recInfo++;
	}

	info->data[t++] = pResult->blockNum;
	recInfo = pResult->block;
	for(index=0;index<pResult->blockNum;index++)
	{
		info->data[t++] = recInfo->x;
		info->data[t++] = recInfo->y;
		recInfo++;
	}
	
	return size;
}

void *GvsstskThrMain(void *pPrm)
{
	AlgObj *pObj = (AlgObj *)pPrm;
	int status;
	unsigned int isQuit = 0;
	AlgResultFrame *pFullFrame;
	Gvss_Result_Str *pResult;
	unsigned int size;	
	int rc;
	
	gGvssQuit = 0;
	if(InitGvss(&gGvss) < 0)
		return (void *)0;

	OSA_attachSignalHandler(SIGINT, Gvss_signalHandler);

	GvsstoPlc info;
	
	while(gGvssQuit == 0)
	{
		if (modbus_connect(gGvss.ctx) == -1) 
		{
			sleep(1);
		}
		
		AppClient_On(TARGET_PLC,pObj);
		
		isQuit = gGvssQuit;
			
		while(isQuit == 0)
		{
			memset(&info,0,sizeof(info));
			status = OSA_queGet(&pObj->ResultFullQue, (Int32 *) & pFullFrame,OSA_TIMEOUT_FOREVER);
			if((status != OSA_SOK)||(pFullFrame == NULL))
			{
				isQuit = 1;
				continue;
			}
			pResult = (Gvss_Result_Str *)(pFullFrame->data);

			size = gvss_dataFormat(pResult,&info); 
			OSA_quePut(&pObj->ResultFreeQue,(Int32)(pFullFrame), OSA_TIMEOUT_FOREVER);
				
			if(size <= GVSSTOPLC_SIZE)
			{
				rc = modbus_write_registers(gGvss.ctx, UT_REGISTERS_ADDRESS,
						UT_REGISTERS_NB, UT_REGISTERS_TAB);
				if (rc != UT_REGISTERS_NB) 
				{
					modbus_close(gGvss.ctx);
					isQuit = 1;
				}
			}	
			else
			{		
				modbus_close(gGvss.ctx);
				isQuit = 1;
			}	
		
			
		}

		AppClient_Off(TARGET_PLC,pObj);
	}

	AppClient_Quit(pObj);
	
	ExitGvss(&gGvss);

	return (void *)0;	
}

