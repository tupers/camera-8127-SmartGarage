#ifndef GVSS_RESULT_STRUCT_TYPE_H
#define GVSS_RESULT_STRUCT_TYPE_H

#define MAX_BLOCK_NUM		50
#define GVSS_MAX_CODELINE		5

typedef struct
{
	int area;
	short x;
	short y;
}Gvss_Result_BlockStr;


typedef struct DETECTRECT_RESULT_STRUCTURE
{
	float position;
	float distance;
	int	  errIndex;
	unsigned char code[GVSS_MAX_CODELINE*2];
	unsigned char reserved[2];
	short top;
	short bottom;
	short left;
	short right;	
	int blockNum;
	Gvss_Result_BlockStr block[MAX_BLOCK_NUM];
} Gvss_Result_Str;
#endif
