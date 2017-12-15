#ifndef REMOTE_DEBUG_MSG_DRV_H
#ifdef __cplusplus
extern "C" {
#endif /* cplusplus */
#define REMOTE_DEBUG_MSG_DRV_H

#define REMOTE_DEBUG_BUF_SIZE	1024

#define REMOTEDEBUG_PORT	9500
#define REMOTEDEBUG_IP		"127.0.0.1"

typedef enum {
	REMOTEDEBUG_MSG_QUIT, 	///< Let remote debug client manager shutdown.
	REMOTEDEBUG_MSG_NONE, 	/// remote debug stop
	REMOTEDEBUG_MSG_SERIAL, /// remote debug with serial (RS232)
	REMOTEDEBUG_MSG_SOCKET, /// remote debug with ethnet (UDP)
	REMOTEDEBUG_MSG_DEBUGINFO, /// 
	REMOTEDEBUG_MSG_MAX 
} RemoteDebugCmd_t;

typedef struct _REMOTEDEBUG_SOCKMSG_BUF{
	RemoteDebugCmd_t	cmd; ///< Message command ID.
	unsigned int dataBytes;
	unsigned char data[REMOTE_DEBUG_BUF_SIZE];
} REMOTEDEBUG_SOCKMSG_BUF;

int InitRemoteDebugMsgDrv(int key);

void CleanupRemoteDebugMsgDrv();

int Remote_printf(char *format, ...);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* REMOTE_DEBUG_MSG_DRV_H */
