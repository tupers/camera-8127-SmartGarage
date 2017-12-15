#ifndef GVSS_PROTOCOL_H
#define GVSS_PROTOCOL_H

#define SOCKET_BACKLOG		5

#define KEEPALIVE				1
#define KEEPIDLE_TIME			3
#define KEEPINTREVAL_TIME		5
#define KEEPCOUNT				3

typedef struct _Gvss_Protocol
{
	int ServerSd;

	int ConnectSd;

	unsigned int port;
	
	char *privateInfo;
	
	int (*init)(struct _Gvss_Protocol *pObj);

	int (*wait_for_client)(struct _Gvss_Protocol *pObj);
	
	int (*send)(struct _Gvss_Protocol *pObj,void *data,unsigned int bytes);

	int (*recv)(struct _Gvss_Protocol *pObj,void *data,unsigned int bytes);
	
	void (*exit)(struct _Gvss_Protocol *pObj);

	int (*get_connectState)(struct _Gvss_Protocol *pObj);
}Gvss_Protocol;

int Gvss_Get_connectState(struct _Gvss_Protocol *pObj);

void Gvss_close_connect(struct _Gvss_Protocol *pObj);

int Gvss_ProtocolInit(struct _Gvss_Protocol *pObj);

int Gvss_ProtocolExit(struct _Gvss_Protocol *pObj);

#endif
