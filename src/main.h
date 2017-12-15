#ifndef MAIN_H
#define MAIN_H

#include <osa_que.h>
#include "osa_sem.h"
#include "gvss_type.h"

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


enum {
	MSG_TYPE_START = 0,

	MSG_TYPE_MSG1, ///< Message type 1. Always reserved for server.
	MSG_TYPE_MSG2, ///< Message type 2.
	MSG_TYPE_MSG3, ///< Message type 3.
	MSG_TYPE_MSG4, ///< Message type 4.
	MSG_TYPE_MSG5, ///< Message type 5.
	MSG_TYPE_MSG6, ///< Message type 6.
	MSG_TYPE_MSG7, ///< Message type 7.
	MSG_TYPE_MSG8, ///< Message type 8.
	MSG_TYPE_MSG9, ///< Message type 9.
	MSG_TYPE_MSG10, ///< Message type 10.
	MSG_TYPE_MSG11, ///< Message type 11.
	MSG_TYPE_MSG12, ///< Message type 12.
	MSG_TYPE_MSG13, ///< Message type 13.
	MSG_TYPE_MSG14, ///< Message type 14.

	MSG_TYPE_END
};

typedef enum 
{
	ALG_CMD_START,
	ALG_CMD_STOP,
	ALG_CMD_RESULT,
	ALG_CMD_DELETE,
}AlgappCmd_t;

typedef struct _ALGAPP_MSG_BUF
{
	long 		Des;
	AlgappCmd_t cmd;
	int			src;
	unsigned	offset;
	unsigned	size;
	int			ret;
}ALGAPP_MSG_BUF;

typedef struct
{
	unsigned int maxSize;
	unsigned int size;
	unsigned char *data;
}AlgResultFrame;

#define MAX_QUE_NUM			5
#define MAX_RESULT_SIZE		2048

typedef enum TARGET_TYPE_
{
	TARGET_PLC,
	TARGET_CLIENT
}TARGET_TYPE;

typedef struct 
{
	int shmid;
	int qid;
	OSA_SemHndl hndl;

	Gvss_Result_Str *pData;
	unsigned dataSize;
	
	OSA_QueHndl ResultFreeQue;
	OSA_QueHndl ResultFullQue;
	AlgResultFrame result[MAX_QUE_NUM];


	OSA_QueHndl ClientFreeQue;
	OSA_QueHndl ClientFullQue;
	AlgResultFrame client[MAX_QUE_NUM];

	unsigned int PlcTarget;
	unsigned int ClientTarget;
}AlgObj;

int AppClient_On(TARGET_TYPE type,AlgObj *pObj);

int AppClient_Off(TARGET_TYPE type,AlgObj *pObj);

int AppClient_Quit(AlgObj *pObj);

#define ALGAPP_SHM_KEY		0xa6a3975
#define ALGAPP_MSG_KEY		0x790518
#define ALGAPP_SHM_SIZE		4906







#endif
