#include "log.h"
#include <stdarg.h>
#include <osa_sem.h>

static FILE* logfp = NULL;
static OSA_SemHndl logHndl;
static OSA_SemHndl logFileHndl;
int log_init()
{
	int status = 0;

	chdir(GVSS_LOGROOT);
	
	if(logfp != NULL)
		return 0;
		
	logfp = fopen(LOG_FILE, "w+");
	if(logfp == NULL)
		return -1;

	status = OSA_semCreate(&logHndl, 1, 1);
	if(status != 0)
	{
		fclose(logfp);

		logfp = NULL;

		return -1;
	}
	
	status = OSA_semCreate(&logFileHndl, 1, 1);
	if(status != 0)
	{
		fclose(logfp);

		OSA_semDelete(&logHndl);
		
		logfp = NULL;

		return -1;
	}
	
	return 0;	
}
	
int log_printf(const char *fmt,...)
{
	int nret = 0;
	long fileByte = 0;
	va_list args;
	struct tm *tblock;
	time_t timer;
		
	if(logfp == NULL)
		return -1;
		
	if(fmt == NULL)
		return -1;	
			
	OSA_semWait(&logHndl, OSA_TIMEOUT_FOREVER);	

	fileByte = ftell(logfp);
	if(fileByte > LOG_FILE_MAX_LEN)
	{
		fclose(logfp);

		logfp = fopen(LOG_FILE, "w+");
	}
	va_start(args,fmt);

	timer = time(NULL);
	tblock = localtime(&timer);

	fprintf(logfp,"%s:",asctime(tblock));
	
	nret = vfprintf(logfp,fmt,args);

	va_end(args);	
	
	OSA_semSignal(&logHndl);
	
	return 0;
}

int log_flush()
{
	OSA_semWait(&logHndl, OSA_TIMEOUT_FOREVER);

	fflush(logfp);
	
	OSA_semSignal(&logHndl);

	return 0;
}

void log_exit()
{
	if(logfp)
	{
		fclose(logfp);
		OSA_semDelete(&logHndl);	
	}
	
	logfp = NULL;
}

static FILE *gvss_logfp = NULL;
static FILE *gvss_errfp = NULL;
char logfileName[50] = {0};
char errfileName[50] = {0};
char lasterrfileName[50] = {0};
unsigned int curlogFileSize = 0;
unsigned int curerrFileSize = 0;
static unsigned int lastPosition = 0;

#include <stdlib.h>
#include <dirent.h>
#include <sys/types.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>
int getlogFileName(char *dir,char *fileName);

int get_file_count(char *pathname)
{
	DIR *dir;
	struct dirent *ptr;
	int ret;
	unsigned int file_count = 0;
	
	ret = access(pathname, F_OK);
	if(ret!=0)
	{
		ret = mkdir(pathname, 0755);
		if(ret != 0)
		{
			return -1;
		}	
	}
	
	dir = opendir(pathname);
	if(dir == NULL)
	{
		perror("opendir");
		return -1;
	}
	
	while((ptr=readdir(dir)) != NULL)
	{
		if(strcmp(ptr->d_name,".") == 0 || strcmp(ptr->d_name,"..") == 0)
			continue;

		file_count++;	
	}	

	closedir(dir);
	
	return file_count;
}

int get_errlogfile_size()
{
	int ret;
	
	ret = get_file_count(GVSS_ERRDIR);
	if(ret < 0 )
		ret = -1;
	else
		ret = ret * ERRLOG_PERFILE_SIZE;

	return ret;		
}

static FILE *openErrLogFile()
{
	int ret = 0;
	if(gvss_errfp != NULL)
		return gvss_errfp;
	else
	{
		curerrFileSize = 0;

		ret = get_errlogfile_size();
		if( ret < 0)
			return NULL;
			
		if(ret >= ERRLOG_FILE_MAXSIZE)
		{
			gvss_errfp = NULL;
		}
		else
		{
			getlogFileName(GVSS_ERRDIR,errfileName);
			
			if(strcmp(errfileName, lasterrfileName) == 0)
			{
				return NULL;				
			}
			else		
			{
				gvss_errfp = fopen(errfileName,"w+");
				if(gvss_errfp == NULL)
					perror("Open Fail");
				else
					strcpy(lasterrfileName, errfileName);	
				
			}
		}		
	}

	return gvss_errfp;	
}

static void closeErrLogFile()
{
	OSA_semWait(&logHndl, OSA_TIMEOUT_FOREVER);

	fclose(gvss_errfp);
	
	gvss_errfp = NULL;

	OSA_semSignal(&logHndl);
}

void flushErrLogFile()
{
	OSA_semWait(&logHndl, OSA_TIMEOUT_FOREVER);
	
	if(gvss_errfp != NULL)
		fflush(gvss_errfp);
		
	OSA_semSignal(&logHndl);
}

int errlogfile_write(Gvss_Result_Str *result)
{
	FILE *fp = NULL;	
	int ret;
	unsigned size = 0;

	if(result == NULL)
		return 0;

	fp = openErrLogFile();
	if(fp == NULL)
	{
		return 0;	
	}
	
	size = sizeof(Gvss_Result_Str) - sizeof(result->block) + sizeof(Gvss_Result_BlockStr) * result->blockNum;
	
	if(size > sizeof(Gvss_Result_Str))
	{
		return 0;
	}	
	ret = fwrite(result, 1, size,fp);
	
	curerrFileSize += size;

	if(curerrFileSize >= ERRLOG_PERFILE_SIZE)
	{
		closeErrLogFile();
	}

	return ret;
}


int get_logfile_size()
{
	int ret;
	
	ret = get_file_count(GVSS_LOGDIR);
	if(ret < 0 )
		ret = -1;
	else
		ret = ret * LOG_PERFILE_SIZE;

	return ret;		
}

int getlogFileName(char *dir,char *fileName)
{
	time_t t;
	struct tm *tp;
	char fileString[30] = {0};
	
	if(fileName == NULL)
		return -1;
	t = time(NULL);
	tp = localtime(&t);	

	memset(fileName,0,50);
	
	sprintf(fileString,"/%d%02d%02d_%02d%02d.log",tp->tm_year + 1900,tp->tm_mon+1,tp->tm_mday,tp->tm_hour,tp->tm_min);	

	strcat(fileName, dir);
	strcat(fileName, fileString);
	return 0;
}

static FILE *openLogFile()
{
	int ret;
	
	if(gvss_logfp != NULL)
		return gvss_logfp;
	else
	{
		curlogFileSize = 0;

		ret = get_logfile_size();
		if( ret < 0)
		{
			printf("get_logfile_size below zero\n");
			return NULL;
		}	
		if(ret >= LOG_FILE_MAXSIZE)
		{
			gvss_logfp = NULL;
		}
		else
		{
			getlogFileName(GVSS_LOGDIR,logfileName);
			
			gvss_logfp = fopen(logfileName,"w+");
			if(gvss_logfp == NULL)
				perror("open");
		}		
	}

	return gvss_logfp;	
}

static void closeLogFile()
{
	if(gvss_logfp != NULL)
		fclose(gvss_logfp);	

	gvss_logfp = NULL;	
}

int logfile_write(Gvss_Result_Str *result)
{
	int ret = 0;
	LogData log;
	FILE *fp;
	
	if(result == NULL)
		return 0;

	log.distance = result->distance;
	log.position = result->position;
		
	fp = openLogFile();
	if(fp != NULL)
	{
#if 0	
		unsigned int size;
		size = sizeof(Gvss_Result_Str) - sizeof(result->block) + sizeof(Gvss_Result_BlockStr) * result->blockNum;
	
		if(size > sizeof(Gvss_Result_Str))
		{
			printf("size too long\n");
			return 0;
		}	
	
		ret = fwrite(result, 1, size,fp);
#endif	
		if((log.position - lastPosition < POSITION_DIFF) && (log.position != 0))
			;
		else
		{			
			ret = fprintf(fp,"%d %d\n",log.distance,log.position);
	
			curlogFileSize += ret;
		
			if(curlogFileSize >= LOG_PERFILE_SIZE)
			{
				closeLogFile();
			}

			lastPosition = log.position;
		}	
	}

	errlogfile_write(result);
	
	if((log.distance == 0) &&(log.position == 0))
	{
//		errlogfile_write(result);
	}
	
	return ret;
}

