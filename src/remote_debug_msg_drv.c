#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <osa_sem.h>
#include <osa_socket.h>
#include "remote_debug_msg_drv.h"



static OSA_SemHndl hndlDrvSem; 
REMOTEDEBUG_SOCKMSG_BUF *remote_buf = NULL;
char *arg_buf = NULL;

static int SendRemoteMsg(char *msgbuf,unsigned int size);

static int sd = -1;

static int SendRemoteMsg(char *msgbuf,unsigned int size)
{
	int ret;

	while(size>0)
	{
		ret = OSA_udp_send_data(sd,"127.0.0.1",REMOTEDEBUG_PORT,msgbuf,size);
		if(ret < 0)
		{
			return -1;
		}
		else
			size -= ret;
	}	

	return 0;
}

int Remote_printf(char *format, ...)
{
	int ret = 0;
    va_list vaArgPtr;
    unsigned int len;

	OSA_semWait(&hndlDrvSem, OSA_TIMEOUT_FOREVER);	
    va_start(vaArgPtr, format);
    vsnprintf(arg_buf, 1024, format, vaArgPtr);
    va_end(vaArgPtr);

	len = strlen(arg_buf);

	if(len < REMOTE_DEBUG_BUF_SIZE)
	{
		memcpy(remote_buf->data,arg_buf,len);
		remote_buf->data[len] = 0;
		remote_buf->cmd = REMOTEDEBUG_MSG_DEBUGINFO;
		remote_buf->dataBytes = len + 1;

		ret = SendRemoteMsg((char *)remote_buf,sizeof(REMOTEDEBUG_SOCKMSG_BUF));
	}	

	OSA_semSignal(&hndlDrvSem);

	return ret;
}
/**
* @brief Initialize file message driver.

* Initialize file message driver. Please see \ref FILE_MSG_DRV_HOW to learn more.
* @note This API must be used before use any other file message driver API.
* @param key [I ] Key number for message queue and share memory.
* @param iProcId [I ] Message ID(Which was define in \ref File_Msg_Def.h) to initialize file message driver.
* @retval 0 Success.
* @retval -1 Fail.
*/
int InitRemoteDebugMsgDrv(int key)
{
	int status = 0;
	status = OSA_semCreate(&hndlDrvSem, 1, 1);
	if(status != 0){
		return -1;
	}

	if((sd = OSA_create_server_socket_udp(0)) < 0){
		OSA_semDelete(&hndlDrvSem);	
		return -1;
	}

	remote_buf = (REMOTEDEBUG_SOCKMSG_BUF *)malloc(sizeof(REMOTEDEBUG_SOCKMSG_BUF));
	if(remote_buf == NULL)
	{
		OSA_semDelete(&hndlDrvSem);	
		return -1;				
	}

	arg_buf = (char *)malloc(2048);
	if(arg_buf == NULL)
	{
		OSA_semDelete(&hndlDrvSem);	
		free(remote_buf);
		return -1;				
	}

	return 0;
}
/**
* @brief Cleanup file message driver.

* This API shoud be called at the end of process.
*/
void CleanupRemoteDebugMsgDrv()
{
	OSA_semDelete(&hndlDrvSem);	
	free(arg_buf);
	free(remote_buf);
}
