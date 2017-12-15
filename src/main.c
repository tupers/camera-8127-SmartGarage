#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/resource.h>
#include "main.h"
#include "osa_msg.h"
#include "osa_shmem.h"
#include "osa_thr.h"
#include "remote_debug_msg_drv.h"
#include "log.h"

static AlgObj	obj;

extern unsigned int gLogDeamonQuit;

int SetEnv(AlgObj *pObj)
{
	int status = OSA_SOK;
	unsigned int idx;
	AlgResultFrame *pInfo;

	obj.PlcTarget = 0;
	obj.ClientTarget = 0;
	
	obj.shmid = OSA_ShareMemInit(ALGAPP_SHM_KEY,ALGAPP_SHM_SIZE);
	if(obj.shmid < 0)
	{
		ERR("OSA_ShareMemInit fail\n");
		return -1;
	}

	obj.qid = OSA_Msg_Init( ALGAPP_MSG_KEY );
	if(obj.qid < 0)
	{
		ERR("OSA_Msg_Init fail\n");

		return -1;
	}

	status = OSA_semCreate(&(obj.hndl), 1, 1);
	if(status != OSA_SOK)
	{
		ERR("OSA_semCreate fail\n");

		return -1;
	}
	
    status = OSA_queCreate(&pObj->ResultFreeQue, MAX_QUE_NUM);
    OSA_assert(status == OSA_SOK);

    status = OSA_queCreate(&pObj->ResultFullQue,MAX_QUE_NUM);
    OSA_assert(status == OSA_SOK);

    for(idx=0;idx<MAX_QUE_NUM;idx++)
    {
		pInfo = &(pObj->result[idx]);
		pInfo->size = 0;
		pInfo->maxSize = MAX_RESULT_SIZE;
		pInfo->data = (unsigned char *)malloc(MAX_RESULT_SIZE);
		OSA_assert(pInfo->data != NULL);

		status = OSA_quePut(&pObj->ResultFreeQue,(Int32)(pInfo), OSA_TIMEOUT_NONE);
		OSA_assert(status == OSA_SOK);
    }


	/*** Init Client queue *********/
    status = OSA_queCreate(&pObj->ClientFreeQue, MAX_QUE_NUM);
    OSA_assert(status == OSA_SOK);

    status = OSA_queCreate(&pObj->ClientFullQue,MAX_QUE_NUM);
    OSA_assert(status == OSA_SOK);

    for(idx=0;idx<MAX_QUE_NUM;idx++)
    {
		pInfo = &(pObj->client[idx]);
		pInfo->size = 0;
		pInfo->maxSize = MAX_RESULT_SIZE;
		pInfo->data = (unsigned char *)malloc(MAX_RESULT_SIZE);
		OSA_assert(pInfo->data != NULL);

		status = OSA_quePut(&pObj->ClientFreeQue,(Int32)(pInfo), OSA_TIMEOUT_NONE);
		OSA_assert(status == OSA_SOK);
    }

    pObj->pData = (Gvss_Result_Str *)malloc(sizeof(Gvss_Result_Str));
    if(NULL == pObj->pData)
    {
		ERR("malloc fail\n");

		return -1;
    }	
	InitRemoteDebugMsgDrv(0);

	log_init();

    if(status != OSA_SOK)
    	return -1;
    	
	return 0;
}

void ClenaEnv(AlgObj *pObj)
{
	unsigned int idx;
	AlgResultFrame *pInfo;
	
	OSA_queDelete(&pObj->ResultFreeQue);

	OSA_queDelete(&pObj->ResultFullQue);

	for(idx=0;idx<MAX_QUE_NUM;idx++)
    {
		pInfo = &(pObj->result[idx]);
		if(pInfo->data)
			free(pInfo->data);
    }

    free(pObj->pData);
}

int AppClient_On(TARGET_TYPE type,AlgObj *pObj)
{
	int ret = 0;
	
	OSA_semWait(&(pObj->hndl), OSA_TIMEOUT_FOREVER);
	if(type == TARGET_PLC)
		pObj->PlcTarget = 1;
	if(type == TARGET_CLIENT)
		pObj->ClientTarget = 1;	

#if 0
	ALGAPP_MSG_BUF msgbuf;
	if((pObj->PlcTarget + pObj->ClientTarget) == 1)
	{
		msgbuf.Des = MSG_TYPE_MSG1;
		msgbuf.src = MSG_TYPE_MSG3;
		msgbuf.cmd = ALG_CMD_START;
		msgbuf.ret = 0;	

		ret = OSA_Msg_Send( pObj->qid, &msgbuf , sizeof(msgbuf));		
	}
#endif	
	OSA_semSignal(&(pObj->hndl));
	
	return ret;
}

int AppClient_Off(TARGET_TYPE type,AlgObj *pObj)
{
	int ret = 0;
	
	OSA_semWait(&(pObj->hndl), OSA_TIMEOUT_FOREVER);
	if(type == TARGET_PLC)
		pObj->PlcTarget = 0;
	if(type == TARGET_CLIENT)
		pObj->ClientTarget = 0;	
#if 0
	ALGAPP_MSG_BUF msgbuf;
	if((pObj->PlcTarget + pObj->ClientTarget) == 0)
	{
		msgbuf.Des = MSG_TYPE_MSG1;
		msgbuf.src = MSG_TYPE_MSG3;
		msgbuf.cmd = ALG_CMD_STOP;
		msgbuf.ret = 0;	

		ret = OSA_Msg_Send( pObj->qid, &msgbuf , sizeof(msgbuf));			
	}
#endif	
	OSA_semSignal(&(pObj->hndl));

	return ret;
}

int AppClient_Start(AlgObj *pObj)
{
	ALGAPP_MSG_BUF msgbuf;
	msgbuf.Des = MSG_TYPE_MSG3;
	msgbuf.src = MSG_TYPE_MSG1;
	msgbuf.cmd = ALG_CMD_START;
	msgbuf.ret = 0;	

	return OSA_Msg_Send( pObj->qid, &msgbuf , sizeof(msgbuf));	
}

int AppClient_Stop(AlgObj *pObj)
{
	ALGAPP_MSG_BUF msgbuf;
	msgbuf.Des = MSG_TYPE_MSG3;
	msgbuf.src = MSG_TYPE_MSG1;
	msgbuf.cmd = ALG_CMD_STOP;
	msgbuf.ret = 0;	

	return OSA_Msg_Send( pObj->qid, &msgbuf , sizeof(msgbuf));	
}

int AppClient_Quit(AlgObj *pObj)
{
	ALGAPP_MSG_BUF msgbuf;
	msgbuf.Des = MSG_TYPE_MSG3;
	msgbuf.src = MSG_TYPE_MSG1;
	msgbuf.cmd = ALG_CMD_DELETE;
	msgbuf.ret = 0;	

	return  OSA_Msg_Send( pObj->qid, &msgbuf , sizeof(msgbuf));	
}


static int ProcMsgTread(void *pPrm)
{
	AlgObj *pObj = (AlgObj *)pPrm;
	ALGAPP_MSG_BUF msgbuf;
	AlgResultFrame *pEmptyFrame;
	int ret;
	int status;
	unsigned int isQuit = 0;
	unsigned int eCount = 0;
	
	msgbuf.ret = 0;

	AppClient_Start(pObj);
	
	while(isQuit == 0)
	{
		ret = OSA_Msg_Rsv( pObj->qid, MSG_TYPE_MSG1, &msgbuf , sizeof(msgbuf));
		if(ret < 0)
		{
			if(eCount > 3)
				isQuit = 1;
			else	
				pObj->qid = OSA_Msg_Init( ALGAPP_MSG_KEY );
			eCount++;
			continue;
		}

		eCount = 0;
		switch(msgbuf.cmd)
		{
			case ALG_CMD_START:
			{
				msgbuf.cmd = ALG_CMD_START;
				break;
			}
			case ALG_CMD_STOP:
			{
				msgbuf.cmd = ALG_CMD_STOP;
				break;
			}
			case ALG_CMD_RESULT:
			{		
				OSA_ShareMemRead(pObj->shmid,msgbuf.offset,pObj->pData, msgbuf.size);		
				pObj->dataSize = msgbuf.size;

				logfile_write(pObj->pData);
				
				OSA_semWait(&(pObj->hndl), OSA_TIMEOUT_FOREVER);
				if(pObj->PlcTarget == 1)
				{
				status = OSA_queGet(&pObj->ResultFreeQue, (Int32 *) &pEmptyFrame,OSA_TIMEOUT_NONE);
				if(status == OSA_SOK)
				{
					if((msgbuf.size + msgbuf.offset) <= pEmptyFrame->maxSize)
					{
						memcpy(pEmptyFrame->data,pObj->pData,pObj->dataSize);
						
						pEmptyFrame->size = pObj->dataSize;
						
//						OSA_ShareMemRead(pObj->shmid,msgbuf.offset,pEmptyFrame->data, msgbuf.size);	

//						pEmptyFrame->size = msgbuf.size;	

						status = OSA_quePut(&pObj->ResultFullQue,(Int32)pEmptyFrame, OSA_TIMEOUT_NONE);
						if(status != OSA_SOK)
						{
							OSA_quePut(&pObj->ResultFreeQue,(Int32)pEmptyFrame, OSA_TIMEOUT_NONE);
						}
					}	
				}	
				}			

				
				
				/*** for Client ***/
				if(pObj->ClientTarget == 1)
				{
				status = OSA_queGet(&pObj->ClientFreeQue, (Int32 *) &pEmptyFrame,OSA_TIMEOUT_NONE);
				if(status == OSA_SOK)
				{
					if((msgbuf.size + msgbuf.offset) <= pEmptyFrame->maxSize)
					{
//						OSA_ShareMemRead(pObj->shmid,msgbuf.offset,pEmptyFrame->data, msgbuf.size);	
//						pEmptyFrame->size = msgbuf.size;	

						memcpy(pEmptyFrame->data,pObj->pData,pObj->dataSize);
						
						pEmptyFrame->size = pObj->dataSize;
						
						status = OSA_quePut(&pObj->ClientFullQue,(Int32)pEmptyFrame, OSA_TIMEOUT_NONE);
						if(status != OSA_SOK)
						{
							OSA_quePut(&pObj->ClientFreeQue,(Int32)pEmptyFrame, OSA_TIMEOUT_NONE);
						}
					}	
				}							
				}
				OSA_semSignal(&(pObj->hndl));
				break;
			}
			case ALG_CMD_DELETE:
			{
				isQuit = 1;
				break;
			}
			default:
			{
				break;
			}
		
		}

		if(msgbuf.src != 0)
		{
			msgbuf.Des = msgbuf.src;
			msgbuf.src = MSG_TYPE_MSG1;
			OSA_Msg_Send( pObj->qid, &msgbuf , sizeof(msgbuf) );
		}
	}

	AppClient_Quit(pObj);

	gLogDeamonQuit = 1;
	return 0;
}

extern void *GvsstskThrMain(void *pPrm);
extern void *GvssClientThrMain(void *pPrm);
extern void *LogDeamonThrMain(void *pPrm);

int main()
{
	int ret;
	OSA_ThrHndl gvsstskHandle;
	OSA_ThrHndl algtskHandle;
	OSA_ThrHndl	logDeamontskHandle;
	
	ret = SetEnv(&obj);	
	if(ret < 0)
	{
		printf("SetEnv Fail\n");
		return -1;
	}

	OSA_thrCreate(&gvsstskHandle, GvsstskThrMain, OSA_THR_PRI_DEFAULT ,OSA_THR_STACK_SIZE_DEFAULT, &obj);

	OSA_thrCreate(&algtskHandle, GvssClientThrMain, OSA_THR_PRI_DEFAULT + 1,OSA_THR_STACK_SIZE_DEFAULT, &obj);

	OSA_thrCreate(&logDeamontskHandle, LogDeamonThrMain, OSA_THR_PRI_DEFAULT + 1,OSA_THR_STACK_SIZE_DEFAULT, &obj);

	ProcMsgTread(&obj);
	
//	OSA_thrCreate(&algtskHandle, ProcMsgTread, OSA_THR_PRI_DEFAULT + 1,OSA_THR_STACK_SIZE_DEFAULT, &obj);
	
	OSA_thrJoin(&gvsstskHandle);

	OSA_thrJoin(&algtskHandle);	
			
	ClenaEnv(&obj);	
	
	return 0;
}
