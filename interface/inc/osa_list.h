#ifndef OSA_LIST_H
#define OSA_LIST_H

typedef struct OSA_LIST
{
	void *privateData;
	struct OSA_LIST *next;
}OsaList;

#define DEF_MAXELEM			10

int OSA_ListCreate(unsigned int maxElem);
void OSA_ListDelete();
int OSA_ListInsert(void *data);
OsaList* OSA_ListGetHead();
OsaList * OSA_ListGetNextElem(OsaList *ListPtr,void **data);
int OSA_ListRemoveElem(OsaList *ListPtr,void *data);
int OSA_ListGetElemNum();
#endif
