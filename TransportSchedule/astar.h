#ifndef _ASTAR_H_
#define _ASTAR_H_

#include <bits/stdc++.h>

#define MaxLength 10000    //�������ȶ��У�Open��������
#define Height     200    //��ͼ�߶�
#define Width      200    //��ͼ���
#define Read_Size 210 

#define Right       (1 << 0)      //��Ӧ������ָ��0-3 ��������
#define Left      (1 << 1)
#define Up       (1 << 2)
#define Down      (1 << 3)

#define GET_PATH  100    //�õ����
#define No_Solution  -100    //�޽������
#define Infinity    0xfffffff

using namespace std;

typedef struct
{
	signed char x, y;
} Point;             //����ṹ��


typedef struct
{
	int x, y;
	unsigned char reachable, sur, value;
} MapNode;

typedef struct Close
{
	MapNode* cur;
	char vis;
	struct Close* from;
	int last_point_search_direction;
	float F, G;
	int H;
} Close;

typedef struct //���ȶ��У�Open��
{
	int length;        //��ǰ���еĳ���
	Close* Array[MaxLength];    //���۽���ָ��
} Open;



// ���ȶ��л�������
void initOpen(Open* q);
void push(Open* q, Close cls[Height][Width], int x, int y, float g);
Close* shift(Open* q);//�õ���ͷ��close* (arrayĩ  

void initGraph(char my_map[Read_Size][Read_Size]);
void init_tempGraph(char temp_map[Read_Size][Read_Size]);
void initClose(Close cls[Height][Width], MapNode graph[Height][Width], int sx, int sy, int dx, int dy);
int astar(Close close[Height][Width], MapNode graph[Height][Width], int sx, int sy, int dx, int dy);


extern MapNode graph[Height][Width];
extern Close close[Height][Width];
extern char my_map[Read_Size][Read_Size];   //�洢map

//��ײ���� ������ʱ��ͼ

extern MapNode temp_g[Height][Width];
extern Close temp_cls[Height][Width];
extern char temp_map[Read_Size][Read_Size];   //�洢map

#endif // !_ASTAR_

