#ifndef _ASTAR_H_
#define _ASTAR_H_

#include <bits/stdc++.h>

#define MaxLength 10000    //用于优先队列（Open表）的数组
#define Height     200    //地图高度
#define Width      200    //地图宽度
#define Read_Size 210 

#define Right       (1 << 0)      //对应机器人指令0-3 右左上下
#define Left      (1 << 1)
#define Up       (1 << 2)
#define Down      (1 << 3)

#define GET_PATH  100    //得到结果
#define No_Solution  -100    //无解决方案
#define Infinity    0xfffffff

using namespace std;

typedef struct
{
	signed char x, y;
} Point;             //方向结构体


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

typedef struct //优先队列（Open表）
{
	int length;        //当前队列的长度
	Close* Array[MaxLength];    //评价结点的指针
} Open;



// 优先队列基本操作
void initOpen(Open* q);
void push(Open* q, Close cls[Height][Width], int x, int y, float g);
Close* shift(Open* q);//拿到队头的close* (array末  

void initGraph(char my_map[Read_Size][Read_Size]);
void init_tempGraph(char temp_map[Read_Size][Read_Size]);
void initClose(Close cls[Height][Width], MapNode graph[Height][Width], int sx, int sy, int dx, int dy);
int astar(Close close[Height][Width], MapNode graph[Height][Width], int sx, int sy, int dx, int dy);


extern MapNode graph[Height][Width];
extern Close close[Height][Width];
extern char my_map[Read_Size][Read_Size];   //存储map

//碰撞处理 所用临时地图

extern MapNode temp_g[Height][Width];
extern Close temp_cls[Height][Width];
extern char temp_map[Read_Size][Read_Size];   //存储map

#endif // !_ASTAR_

