#include "osa_list.h"
#include "osa_sem.h"

static OSA_SemHndl semHndl;
static OsaList		Head;
static unsigned int maxListNum = 0;
static unsigned int ListElemCount = 0;

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

	ListElemCount = 0;
	
	return ret;	
}

void OSA_ListDelete()
{
	OsaList *cur;
	OSA_semWait(&semHndl, OSA_TIMEOUT_FOREVER);
	OsaList *ptr = Head.next;
	while(1)
	{
		if(ptr)
		{
			cur = ptr->next;
			if(ptr->privateData)
				free(ptr->privateData);
			free(ptr);	
		}
		else
			break;
			
		ptr = ptr->next;
	}

	ListElemCount = 0;
	
	OSA_semSignal(&semHndl);
	
	OSA_semDelete(&semHndl);
}

int OSA_ListGetElemNum()
{
	unsigned int count;
	OSA_semWait(&semHndl, OSA_TIMEOUT_FOREVER);

	count =  ListElemCount;
	
	OSA_semSignal(&semHndl);

	return count;
}
int OSA_ListInsert(void *data)
{
	int ret = OSA_SOK;
	OsaList *ptr;
	OsaList *cur;
	unsigned int elemCount = 0;
	
	OSA_semWait(&semHndl, OSA_TIMEOUT_FOREVER);
	ptr = &Head;
	while(1)
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
		
		cur->privateData = data;
		cur->next = NULL;

		ptr->next = cur;	

		ListElemCount++;	
	}
	else
		ret = OSA_EFAIL;	
	OSA_semSignal(&semHndl);
	
	return ret;	
}

OsaList* OSA_ListGetHead()
{
	return &Head;
}

OsaList* OSA_ListGetNextElem(OsaList *ListPtr,void **data)
{
	OsaList *cur = NULL;
	
	OSA_semWait(&semHndl, OSA_TIMEOUT_FOREVER);
	if(ListPtr == NULL)
	{
		*data = NULL;
		
		OSA_semSignal(&semHndl);

		return NULL;
	}	
	
	cur = ListPtr->next;
	if(cur == NULL)
	{
		*data = NULL;	
	}
	else
	{
		*data = cur->privateData;
	}	
	
	OSA_semSignal(&semHndl);

	return cur;	
}

int OSA_ListRemoveElem(OsaList *ListPtr,void *data)
{
	OsaList *cur = NULL;
	OsaList *ptr = Head.next;
	OsaList *prev = &Head;
	
	OSA_semWait(&semHndl, OSA_TIMEOUT_FOREVER);

	while(1)
	{	
		cur = ptr;
		if(cur == NULL)
			break;
			
		if(ListPtr != NULL)
		{
			if(cur == ListPtr)
			{
				prev->next = cur->next;
				if(cur->privateData)
					free(cur->privateData);
					
				free(cur);

				ListElemCount--;
				break;
			}
		}
		else
		{
			if(data != NULL)
			{
				if((unsigned int)data == (unsigned int)(cur->privateData))
				{
					prev->next = cur->next;
					if(cur->privateData)
						free(cur->privateData);
					
					free(cur);

					ListElemCount--;
					break;				
				}			
			}

		}
		
		prev = ptr;
		ptr = ptr->next;
	}

	
	OSA_semSignal(&semHndl);

	return cur;	
}
