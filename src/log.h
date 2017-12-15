#ifndef GVSS_LOG_H
#define GVSS_LOG_H

#include "gvss_type.h"

#define LOG_FILE					"/opt/logfile"
#define LOG_FILE_MAX_LEN			(10*1024*1024)


#define POSITION_DIFF				0			//mm
#define	LOG_PERFILE_SIZE			0x100000		//1 MBytes
#define LOG_FILE_MAXSIZE			0xA00000
	

#define ERRLOG_PERFILE_SIZE			0x100000				//1M Bytes
#define ERRLOG_FILE_MAXSIZE			0xA00000				//10M Bytes
								
#define GVSS_LOGDIR					"gvss_log"
#define GVSS_ERRDIR					"gvss_err"
#define GVSS_LOGROOT				"/tmp/nand/"


typedef struct _LOGFILE_OBJ
{
	unsigned int log_totalSize;
	unsigned int err_totalSize;
}LogFileObj;

typedef struct _LOG_DATA
{
	unsigned distance;
	unsigned position;
}LogData;

typedef struct
{
	unsigned int area;
	short xcoord;
	short ycoord;
}GvssLogRecInfo;

typedef struct _ERR_DATA
{
	int	  errIndex;
	unsigned char code[6];
	unsigned char reserved[2];
	short top;
	short bottom;
	short left;
	short right;
	unsigned RecNum;
	GvssLogRecInfo recInfo[0]; 	
}ErrData;


int log_init();

int log_printf(const char *fmt,...);

void log_exit();

int log_flush();

int logfile_write(Gvss_Result_Str *log);

#endif
