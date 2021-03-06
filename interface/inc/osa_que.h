

#ifndef _OSA_QUE_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#define _OSA_QUE_H_

#include <osa.h>

typedef struct {

  Uint32 curRd;
  Uint32 curWr;
  Uint32 len;
  Uint32 count;

  Int32 *queue;

  pthread_mutex_t lock;
  pthread_cond_t  condRd;
  pthread_cond_t  condWr;
  
} OSA_QueHndl;

#define MAX_QUE_NUM			5

int OSA_queCreate(OSA_QueHndl *hndl, Uint32 maxLen);
int OSA_queDelete(OSA_QueHndl *hndl);
int OSA_quePut(OSA_QueHndl *hndl, Int32  value, Uint32 timeout);
int OSA_queGet(OSA_QueHndl *hndl, Int32 *value, Uint32 timeout);
int OSA_quePeek(OSA_QueHndl *hndl, Int32 *value);
Uint32 OSA_queGetQueuedCount(OSA_QueHndl *hndl);
Bool OSA_queIsEmpty(OSA_QueHndl *hndl);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _OSA_QUE_H_ */



