#include "astar.h"

MapNode graph[Height][Width];
Close close[Height][Width];
char my_map[Read_Size][Read_Size];   //存储map
//碰撞处理 所用临时地图

MapNode temp_g[Height][Width];
Close temp_cls[Height][Width];
char temp_map[Read_Size][Read_Size];   //存储map

const Point dir[4] =           //对4个方向赋值
{
	{0, 1},   // Right
	{0, -1},  // Left
	{-1, 0},  // Up
	{1, 0},   // Down
};


// 优先队列基本操作
void initOpen(Open* q)    //优先队列初始化
{
	q->length = 0;        // 队内元素数初始为0
}

void push(Open* q, Close cls[Height][Width], int x, int y, float g)
{    //向优先队列（Open表）中添加元素
	Close* t;
	int i, mintag;
	cls[x][y].G = g;    //所添加节点的现有代价
	cls[x][y].F = cls[x][y].G + cls[x][y].H;   //计算预估代价
	q->Array[q->length++] = &(cls[x][y]);
	mintag = q->length - 1;
	for (i = 0; i < q->length - 1; i++)
	{
		if (q->Array[i]->F < q->Array[mintag]->F)
		{
			mintag = i;
		}
	}       //获取最小f的下标
	t = q->Array[q->length - 1];
	q->Array[q->length - 1] = q->Array[mintag];
	q->Array[mintag] = t;    //将评价函数值最小节点置于队头（array末
}

Close* shift(Open* q)           //拿到队头的close* (array末
{
	return q->Array[--q->length];
}


// 地图初始化操作
//由map生成初始graph表 该表表示了mapnode间的邻接与可达情况
void initGraph(char my_map[Read_Size][Read_Size])
{
	int i, j;
	for (i = 0; i < Height; i++)
	{
		for (j = 0; j < Width; j++)
		{
			graph[i][j].x = i; //地图坐标X
			graph[i][j].y = j; //地图坐标Y
			if ((my_map[i][j] == '.') || (my_map[i][j] == 'B') || (my_map[i][j] == 'A'))
				graph[i][j].value = 0;
			else graph[i][j].value = 1;
			graph[i][j].reachable = (graph[i][j].value == 0);    // 节点可到达性  value=0 可达
			graph[i][j].sur = 0; //邻接节点
			if (!graph[i][j].reachable)
			{
				continue;
			}
			if (j > 0)
			{
				if (graph[i][j - 1].reachable)    // left节点可以到达
				{
					graph[i][j].sur |= Left;
					graph[i][j - 1].sur |= Right;
				}
			}
			if (i > 0)
			{
				if (graph[i - 1][j].reachable)    // up节点可以到达
				{
					graph[i][j].sur |= Up;
					graph[i - 1][j].sur |= Down;
				}
			}
		}
	}
}

//临时地图 碰撞用
void init_tempGraph(char temp_map[Read_Size][Read_Size])
{
	int i, j;
	for (i = 0; i < Height; i++)
	{
		for (j = 0; j < Width; j++)
		{
			temp_g[i][j].x = i; //地图坐标X
			temp_g[i][j].y = j; //地图坐标Y
			if ((temp_map[i][j] == '.') || (temp_map[i][j] == 'B') || (temp_map[i][j] == 'A'))
				temp_g[i][j].value = 0;
			else temp_g[i][j].value = 1;
			temp_g[i][j].reachable = (temp_g[i][j].value == 0);    // 节点可到达性  value=0 可达
			temp_g[i][j].sur = 0; //邻接节点
			if (!temp_g[i][j].reachable)
			{
				continue;
			}
			if (j > 0)
			{
				if (temp_g[i][j - 1].reachable)    // left节点可以到达
				{
					temp_g[i][j].sur |= Left;
					temp_g[i][j - 1].sur |= Right;
				}
			}
			if (i > 0)
			{
				if (temp_g[i - 1][j].reachable)    // up节点可以到达
				{
					temp_g[i][j].sur |= Up;
					temp_g[i - 1][j].sur |= Down;
				}
			}
		}
	}
}

//由graph生成close表 该表与graph大小一样 显示了各点是否被访问 路径的上一点 并用于计算F  G   H:预估代价
void initClose(Close cls[Height][Width], MapNode graph[Height][Width], int sx, int sy, int dx, int dy)
{
	int i, j;
	for (i = 0; i < Height; i++)
	{
		for (j = 0; j < Width; j++)
		{
			cls[i][j].cur = &graph[i][j];        // Close表所指节点
			cls[i][j].vis = !graph[i][j].reachable;        // 是否被访问
			cls[i][j].from = NULL;                // 所来节点
			cls[i][j].G = cls[i][j].F = 0;
			cls[i][j].H = abs(dx - i) + abs(dy - j);    // H是预估代价
		}
	}
	cls[sx][sy].F = cls[sx][sy].H;            //起始点评价初始值
	//    cls[sy][sy].G = 0;                        //移步花费代价值
	cls[dx][dy].G = Infinity;   //
}

// 打乱搜寻方向 
// A*算法遍历
int astar(Close close[Height][Width], MapNode graph[Height][Width], int sx, int sy, int dx, int dy)
{
	int  curX, curY, surX, surY;
	float surG;
	Open q; //Open表
	Close* p;

	initOpen(&q);
	initClose(close, graph, sx, sy, dx, dy);
	close[sx][sy].vis = 1;
	push(&q, close, sx, sy, 0);  //把起点加入优先队列
	vector<int> i_list = { 0,1,2,3 };
	while (q.length)
	{
		p = shift(&q);
		curX = p->cur->x;
		curY = p->cur->y;
		if (!p->H)
		{
			return GET_PATH;     //有结果 
		}
		// Right       (1 << 0)   i=0向右搜        
		// Left      (1 << 1)    i=1       
		// Up       (1 << 2)    i=2  
		// Down      (1 << 3)     i=3    
		random_shuffle(i_list.begin(), i_list.end());
		for (int i : i_list) //搜寻sur  
		{
			if (!(p->cur->sur & (1 << i))) //检查可达性
			{
				continue;
			}
			surX = curX + dir[i].x;
			surY = curY + dir[i].y;
			if (!close[surX][surY].vis)  //检查是否已经被访问
			{
				close[surX][surY].vis = 1;
				close[surX][surY].from = p;              //记录上一个点
				close[surX][surY].last_point_search_direction = i;
				surG = p->G + abs(curX - surX) + abs(curY - surY); //改成abs
				push(&q, close, surX, surY, surG);
			}
		}
	}
	return No_Solution; //无结果
}


