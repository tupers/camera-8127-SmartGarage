#include "gvssClientApp.h"
#include "main.h"
#include "gvssProtocol.h"
#include "remote_debug_msg_drv.h"
#include "log.h"
#include "gvss_type.h"		

static GvssClientObj gGvssClient;
static unsigned int gGvssClientQuit = 0;

static int InitGvssClient(GvssClientObj *pObj)
{
	int ret = 0;

	pObj->Com.port = CLIENTPORT;
	ret = Gvss_ProtocolInit(&(pObj->Com));
	if(ret < 0)
	{
		ERR("Gvss_ProtocolInit fail\n");

		return -1;
	}
	
	return 0;
}

static void ExitGvssClient(GvssClientObj *pObj)
{
	Gvss_ProtocolExit(&(pObj->Com));

	log_exit();	
}


void GvssClient_signalHandler(int signum)
{
    gGvssClientQuit = 1;

	printf("catch signal\n");
}

void PrintResult(Gvss_Result_Str *result)
{
	int i;
	int rectNum = result->blockNum;
	printf("rectNum=%d\n",rectNum);

	for(i=0;i<rectNum;i++)
	{
		printf("area=%d,x=%d,y=%d\n",result->block[i].area,result->block[i].x,result->block[i].y);
	}

	printf("top:%d,bottom:%d,left:%d,right:%d\n",result->top,result->bottom,result->left,result->right);	
}
void *GvssClientThrMain(void *pPrm)
{
	AlgObj *pObj = (AlgObj *)pPrm;
	int status;
	unsigned int isQuit = 0;
	AlgResultFrame *pFullFrame;
	Gvss_Result_Str *pResult;
	unsigned int size;	

	gGvssClientQuit = 0;
	if(InitGvssClient(&gGvssClient) < 0)
		return (void *)0;

	OSA_attachSignalHandler(SIGINT, GvssClient_signalHandler);

	pResult = (Gvss_Result_Str *)malloc(sizeof(Gvss_Result_Str));
	if(NULL == pResult)
	{
		printf("malloc fail\n");

		return NULL;
	}
	
	while(gGvssClientQuit == 0)
	{
		status = gGvssClient.Com.wait_for_client(&(gGvssClient.Com));
		if(status < 0)
		{
			gGvssClientQuit = 1;
			isQuit = 1;
		}

		AppClient_On(TARGET_CLIENT,pObj);
		
		isQuit = gGvssClientQuit;
					
		while((isQuit == 0)&&Gvss_Get_connectState(&(gGvssClient.Com)))
		{
			status = OSA_queGet(&pObj->ClientFullQue, (Int32 *) & pFullFrame,OSA_TIMEOUT_FOREVER);
			if((status != OSA_SOK)||(pFullFrame == NULL))
			{
				isQuit = 1;
				continue;
			}

			memcpy(pResult,pFullFrame->data,pFullFrame->size);
			size = pFullFrame->size;
			
			OSA_quePut(&pObj->ClientFreeQue,(Int32)(pFullFrame), OSA_TIMEOUT_FOREVER);

//			PrintResult((Gvss_Result_Str *)pResult);	
			if(Gvss_Get_connectState(&(gGvssClient.Com)))
			{
				status = gGvssClient.Com.send(&(gGvssClient.Com),pResult,size);				
				if(status < 0)
				{
					isQuit = 1;
				}

			}	
			else
			{		
				isQuit = 1;
			}		
		}
		
		AppClient_Off(TARGET_CLIENT,pObj);

		Gvss_close_connect(&(gGvssClient.Com));
	}

	free(pResult);
	
	AppClient_Quit(pObj);
	
	ExitGvssClient(&gGvssClient);

	return (void *)0;	
}

