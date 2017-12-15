#ifndef OSA_MSG_H
#define OSA_MSG_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include <osa.h>


int OSA_Msg_Init( int msgKey );
int OSA_Msg_Send( int qid, void *pdata , int size );
int OSA_Msg_Rsv( int qid, int msg_type, void *pdata , int size );
int OSA_Msg_Send_Rsv( int qid, int msg_type, void *pdata , int size );
int OSA_Msg_Kill( int qid );

#endif
