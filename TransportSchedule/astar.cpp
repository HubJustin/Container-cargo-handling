#include "astar.h"

MapNode graph[Height][Width];
Close close[Height][Width];
char my_map[Read_Size][Read_Size];   //�洢map
//��ײ���� ������ʱ��ͼ

MapNode temp_g[Height][Width];
Close temp_cls[Height][Width];
char temp_map[Read_Size][Read_Size];   //�洢map

const Point dir[4] =           //��4������ֵ
{
	{0, 1},   // Right
	{0, -1},  // Left
	{-1, 0},  // Up
	{1, 0},   // Down
};


// ���ȶ��л�������
void initOpen(Open* q)    //���ȶ��г�ʼ��
{
	q->length = 0;        // ����Ԫ������ʼΪ0
}

void push(Open* q, Close cls[Height][Width], int x, int y, float g)
{    //�����ȶ��У�Open�������Ԫ��
	Close* t;
	int i, mintag;
	cls[x][y].G = g;    //����ӽڵ�����д���
	cls[x][y].F = cls[x][y].G + cls[x][y].H;   //����Ԥ������
	q->Array[q->length++] = &(cls[x][y]);
	mintag = q->length - 1;
	for (i = 0; i < q->length - 1; i++)
	{
		if (q->Array[i]->F < q->Array[mintag]->F)
		{
			mintag = i;
		}
	}       //��ȡ��Сf���±�
	t = q->Array[q->length - 1];
	q->Array[q->length - 1] = q->Array[mintag];
	q->Array[mintag] = t;    //�����ۺ���ֵ��С�ڵ����ڶ�ͷ��arrayĩ
}

Close* shift(Open* q)           //�õ���ͷ��close* (arrayĩ
{
	return q->Array[--q->length];
}


// ��ͼ��ʼ������
//��map���ɳ�ʼgraph�� �ñ��ʾ��mapnode����ڽ���ɴ����
void initGraph(char my_map[Read_Size][Read_Size])
{
	int i, j;
	for (i = 0; i < Height; i++)
	{
		for (j = 0; j < Width; j++)
		{
			graph[i][j].x = i; //��ͼ����X
			graph[i][j].y = j; //��ͼ����Y
			if ((my_map[i][j] == '.') || (my_map[i][j] == 'B') || (my_map[i][j] == 'A'))
				graph[i][j].value = 0;
			else graph[i][j].value = 1;
			graph[i][j].reachable = (graph[i][j].value == 0);    // �ڵ�ɵ�����  value=0 �ɴ�
			graph[i][j].sur = 0; //�ڽӽڵ�
			if (!graph[i][j].reachable)
			{
				continue;
			}
			if (j > 0)
			{
				if (graph[i][j - 1].reachable)    // left�ڵ���Ե���
				{
					graph[i][j].sur |= Left;
					graph[i][j - 1].sur |= Right;
				}
			}
			if (i > 0)
			{
				if (graph[i - 1][j].reachable)    // up�ڵ���Ե���
				{
					graph[i][j].sur |= Up;
					graph[i - 1][j].sur |= Down;
				}
			}
		}
	}
}

//��ʱ��ͼ ��ײ��
void init_tempGraph(char temp_map[Read_Size][Read_Size])
{
	int i, j;
	for (i = 0; i < Height; i++)
	{
		for (j = 0; j < Width; j++)
		{
			temp_g[i][j].x = i; //��ͼ����X
			temp_g[i][j].y = j; //��ͼ����Y
			if ((temp_map[i][j] == '.') || (temp_map[i][j] == 'B') || (temp_map[i][j] == 'A'))
				temp_g[i][j].value = 0;
			else temp_g[i][j].value = 1;
			temp_g[i][j].reachable = (temp_g[i][j].value == 0);    // �ڵ�ɵ�����  value=0 �ɴ�
			temp_g[i][j].sur = 0; //�ڽӽڵ�
			if (!temp_g[i][j].reachable)
			{
				continue;
			}
			if (j > 0)
			{
				if (temp_g[i][j - 1].reachable)    // left�ڵ���Ե���
				{
					temp_g[i][j].sur |= Left;
					temp_g[i][j - 1].sur |= Right;
				}
			}
			if (i > 0)
			{
				if (temp_g[i - 1][j].reachable)    // up�ڵ���Ե���
				{
					temp_g[i][j].sur |= Up;
					temp_g[i - 1][j].sur |= Down;
				}
			}
		}
	}
}

//��graph����close�� �ñ���graph��Сһ�� ��ʾ�˸����Ƿ񱻷��� ·������һ�� �����ڼ���F  G   H:Ԥ������
void initClose(Close cls[Height][Width], MapNode graph[Height][Width], int sx, int sy, int dx, int dy)
{
	int i, j;
	for (i = 0; i < Height; i++)
	{
		for (j = 0; j < Width; j++)
		{
			cls[i][j].cur = &graph[i][j];        // Close����ָ�ڵ�
			cls[i][j].vis = !graph[i][j].reachable;        // �Ƿ񱻷���
			cls[i][j].from = NULL;                // �����ڵ�
			cls[i][j].G = cls[i][j].F = 0;
			cls[i][j].H = abs(dx - i) + abs(dy - j);    // H��Ԥ������
		}
	}
	cls[sx][sy].F = cls[sx][sy].H;            //��ʼ�����۳�ʼֵ
	//    cls[sy][sy].G = 0;                        //�Ʋ����Ѵ���ֵ
	cls[dx][dy].G = Infinity;   //
}

// ������Ѱ���� 
// A*�㷨����
int astar(Close close[Height][Width], MapNode graph[Height][Width], int sx, int sy, int dx, int dy)
{
	int  curX, curY, surX, surY;
	float surG;
	Open q; //Open��
	Close* p;

	initOpen(&q);
	initClose(close, graph, sx, sy, dx, dy);
	close[sx][sy].vis = 1;
	push(&q, close, sx, sy, 0);  //�����������ȶ���
	vector<int> i_list = { 0,1,2,3 };
	while (q.length)
	{
		p = shift(&q);
		curX = p->cur->x;
		curY = p->cur->y;
		if (!p->H)
		{
			return GET_PATH;     //�н�� 
		}
		// Right       (1 << 0)   i=0������        
		// Left      (1 << 1)    i=1       
		// Up       (1 << 2)    i=2  
		// Down      (1 << 3)     i=3    
		random_shuffle(i_list.begin(), i_list.end());
		for (int i : i_list) //��Ѱsur  
		{
			if (!(p->cur->sur & (1 << i))) //���ɴ���
			{
				continue;
			}
			surX = curX + dir[i].x;
			surY = curY + dir[i].y;
			if (!close[surX][surY].vis)  //����Ƿ��Ѿ�������
			{
				close[surX][surY].vis = 1;
				close[surX][surY].from = p;              //��¼��һ����
				close[surX][surY].last_point_search_direction = i;
				surG = p->G + abs(curX - surX) + abs(curY - surY); //�ĳ�abs
				push(&q, close, surX, surY, surG);
			}
		}
	}
	return No_Solution; //�޽��
}


