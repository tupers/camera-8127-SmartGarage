#include "osa_list.h"
#include "osa_sem.h"

static OSA_SemHndl semHndl;
static OsaList		Head;
static unsigned int maxListNum = 0;

int OSA_ListCreate(unsigned int maxElem)
{
	int ret = OSA_SOK;
	
	ret = OSA_semCreate(&semHndl, 1, 1);
	if(ret != OSA_SOK)
		return OSA_EFAIL;
	if(maxElem == 0)
		maxListNum = DEF_MAXELEM;
	else
		maxListNum = maxElem;
		
	Head.privateData = NULL;
	Head.next = NULL;
	
	return ret;	
}

void OSA_ListDelete()
{
	int ret = OSA_SOK;
	OsaList *cur;
	OSA_semWait(&semHndl, OSA_TIMEOUT_FOREVER);
	OsaList *ptr = Head->next;
	while(true)
	{
		if(ptr)
		{
			cur = ptr->next
			if(ptr->privateData)
				free(ptr->privateData);
			free(ptr);	
		}
		else
			break;
			
		ptr = ptr->next;
	}

	OSA_semSignal(&semHndl);
	
	OSA_semDelete(OSA_SemHndl *hndl);
	
	return ret;	
}
int OSA_ListInsert(void *data)
{
	int ret = OSA_SOK;
	OsaList *ptr;
	OsaList *cur;
	unsigned int elemCount = 0;
	
	OSA_semWait(&semHndl, OSA_TIMEOUT_FOREVER);
	ptr = Head;
	while(true)
	{
		if(ptr->next == NULL)
		{
			break;	
		}
		else
		{
			ptr = ptr->next;
			elemCount++;
		}
	}

	if(elemCount < maxListNum)
	{
		cur = (OsaList *)malloc(sizeof(OsaList));
		if(NULL == cur)
		{
			OSA_semSignal(&semHndl);
			return OSA_EFAIL;
		}
	}

	cur->privateData = data;
	cur->next = NULL;

	ptr->next = cur;
	
	OSA_semSignal(&semHndl);
	return ret;	
}

OsaList * OSA_ListGetNext(OsaList *ListPtr,void *data)
{
	int ret = OSA_SOK;
	OsaList *cur = NULL;
	
	OSA_semWait(&semHndl, OSA_TIMEOUT_FOREVER);
	if(ListPtr == NULL)
	{
		cur = Head->next;
		if(cur == NULL)
			data = NULL;
	}
	else
	{
		cur = ListPtr->next;
		if(cur == NULL)
		{
			data = NULL;	
		}
		else
		{
			data = cur->privateData;
		}	
	}
	
	OSA_semSignal(&semHndl);

	return cur;	
}
