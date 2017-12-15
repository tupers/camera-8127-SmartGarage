#include <stdio.h>
#include <stdlib.h>
#include "gvssApp.h"
#include "osa_socket.h"

#define DEBUG

#ifdef DEBUG
#define printf(fmt, args...)			\
do {									\
	printf("%s @ Line : %d in %s :",__func__,__LINE__,__FILE__);	\
	printf("Debug " fmt, ##args);		\
}while(0)								\

#else
#define printf(fmt, args...)
#endif

#define ERR(fmt, args...)				\
do {									\
	printf("%s @ Line : %d in %s :",__func__,__LINE__,__FILE__);	\
	printf("Error " fmt, ##args);		\
}while(0)								\

static int Gvss_Init(struct _Gvss_Protocol *pObj)
{
	if(pObj->ServerSd > 0)
		return 0;
		
	pObj->ServerSd = OSA_create_server_socket(pObj->port,SOCKET_BACKLOG);
	if(pObj->ServerSd < 0)
	{
		ERR("OSA_create_server_socket Fail\n");

		return -1;
	}

	return 0;
}

static int Gvss_wait_for_client(struct _Gvss_Protocol *pObj)
{
	int ret;
	
	if(pObj->ServerSd < 0)
		return -1;
		
	pObj->ConnectSd = OSA_server_accept(pObj->ServerSd,(struct sockaddr *)(pObj->privateInfo));
	if(pObj->ConnectSd < 0)
	{
		ERR("OSA_server_accept\n");

		return -1;
	}

	ret = OSA_Set_keepalive(pObj->ConnectSd,KEEPALIVE,KEEPINTREVAL_TIME,KEEPIDLE_TIME,KEEPCOUNT);
	if(ret<0)
	{
		ERR("OSA_Set_keepalive fail\n");
	}

	ret = OSA_Set_Nodelay(pObj->ConnectSd);
	if(ret<0)
	{
		ERR("OSA_Set_Nodelay fail\n");
	}

	return 0;
}

static int Gvss_send(struct _Gvss_Protocol *pObj,void *data,unsigned int bytes)
{
	int ret;

	if(pObj->ConnectSd == -1)
		return -1;
		
	ret = OSA_socket_send_data(pObj->ConnectSd,data,bytes);
	if(ret < 0)
		return -1;

	return 0;	
	
}

static int Gvss_recv(struct _Gvss_Protocol *pObj,void *data,unsigned int bytes)
{
	if(pObj->ConnectSd == -1)
		return -1;
		
	return OSA_socket_recv_data(pObj->ConnectSd,data,bytes);
}
	
static void Gvss_exit(struct _Gvss_Protocol *pObj)
{
	if(pObj->ConnectSd)
		OSA_delete_server_socket(pObj->ConnectSd);	

	if(pObj->ServerSd)
		OSA_delete_server_socket(pObj->ServerSd);	

	pObj->ConnectSd = -1;
	
	pObj->ServerSd = -1;			
}

void Gvss_close_connect(struct _Gvss_Protocol *pObj)
{
	close(pObj->ConnectSd);
}

int Gvss_Get_connectState(struct _Gvss_Protocol *pObj)
{
	if(pObj->ConnectSd == -1)
		return 0;
		
	return OSA_socket_get_tcpconnect_state(pObj->ConnectSd);
}

int Gvss_ProtocolInit(struct _Gvss_Protocol *pObj)
{
	pObj->ServerSd = -1;

	pObj->privateInfo = (char *)malloc(sizeof(struct sockaddr));
	if(pObj->privateInfo == NULL)
		return -1;
		
	pObj->init = Gvss_Init;

	pObj->wait_for_client = Gvss_wait_for_client;
	
	pObj->send = Gvss_send;

	pObj->recv = Gvss_recv;

	pObj->exit = Gvss_exit;

	pObj->get_connectState = Gvss_Get_connectState;
	pObj->init(pObj);

	return 0;
}

int Gvss_ProtocolExit(struct _Gvss_Protocol *pObj)
{
	if(pObj->privateInfo)
		free(pObj->privateInfo);

	pObj->exit(pObj);
	
	pObj->ServerSd = -1;
	
	pObj->ConnectSd = -1;
		
	pObj->init = NULL;
	
	pObj->send = NULL;
	
	pObj->recv = NULL;
	
	pObj->exit = NULL;

	pObj->wait_for_client = NULL;

	return 0;
}
