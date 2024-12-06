#define _CRT_SECURE_NO_WARNINGS 1
#include <bits/stdc++.h>
#include<iostream>
#include <stdio.h>
#include<fstream>
#include "astar.h"
#define No_Dst  0
#define Dst_Goods  1
#define Dst_Berth   2

#define Berth_NoGoods -200

#define No_dst_berth  -50
#define Find_dst_berth 50

#define No_Crash     20
#define Crash_type1  21
#define Crash_type2  22
#define Crash_type3  23
#define Meet_Bad     24
#define Robot_wait   30

using namespace std;
static int goods_id = 0;
//��������洢  ����������ÿ100֡����15����Ʒ
struct single_goods {
	int id;          //id��Ψһ��  ������Ʒ����
	int x, y, value;
	int start_zhen;
};
struct single_goods No_goods = { -1 };
vector<single_goods> existing_goods = {};

//������ ��λ�����Ķ���
const int robot_num = 10;
const int berth_num = 10;

struct Robot
{
	int have_goods, x, y, status;
	int dst_x;
	int	dst_y;   //�յ�
	int dst_berth_id;
	int dst_type = No_Dst;
	single_goods dst_goods = No_goods;
	Robot() {}
	Robot(int startX, int startY) {
		x = startX;
		y = startY;
	}
}robot[robot_num];  //10�������� 0-9

struct Berth
{
	int id;
	int x;      //���Ͻ�����
	int y;
	int transport_time;
	int loading_speed;
	int now_loading = 0;         //0��ʾû�д� 1��ʾ�д���װж�� �����д���ȥ��
	int goods_num;  //��λ������Ʒ��   
	Berth() {}
	Berth(int x, int y, int transport_time, int loading_speed) {
		this->x = x;
		this->y = y;
		this->transport_time = transport_time;
		this->loading_speed = loading_speed;
	}
}berth[berth_num];  //10����λ 0-9

int boat_capacity;
struct Boat
{
	int goods_num;  //�������л�������
	int pos, status;
	int task_berth = -1;  //��ʼ״̬
	int task_next_berth = -1;
	int stay_start;
	int arrive_flag = -1;
}boat[5];   //�����Ҵ�  0-4


//������·���洢����
struct Path_Points {
	int x, y;
};
struct Robot_Path {
	vector<Path_Points>  path_points;      //·��������ָ������1  
	vector<int> commands;   //ָ��           �ǵݹ�汾ָ���·�����Ŵ��
	int all_steps;        //�ܲ���
	int cur_step = 0;   //��ǰ����  ����Ϊ0
}robot_path[10];      //10����ά�����Ӧ10�������˵�·�� {{x,y,command},....} 

//�ݹ�
void rec_path(Close* p, int robot_id) {
	if (p->from->from != NULL) {
		p = p->from;
		rec_path(p, robot_id);
		robot_path[robot_id].path_points.push_back({ p->cur->x,p->cur->y });
		robot_path[robot_id].commands.push_back(p->last_point_search_direction);
	}
}

int get_path(Close close[Height][Width], MapNode graph[Height][Width], int robot_id, int sx, int sy, int dx, int dy) {
	int res;
	res = astar(close, graph, sx, sy, dx, dy);
	if (res == No_Solution) return No_Solution;
	else {
		Close* p = &(close[dx][dy]);
		robot_path[robot_id].cur_step = 0;
		robot_path[robot_id].path_points.push_back({ sx,sy });
		rec_path(p, robot_id);
		robot_path[robot_id].path_points.push_back({ p->cur->x,p->cur->y });
		robot_path[robot_id].commands.push_back(p->last_point_search_direction);
		robot_path[robot_id].all_steps = robot_path[robot_id].commands.size();
		return GET_PATH;
	}
}

//�����յ��·�����  ������״̬�ع�ԭʼ
void clear_path(int robot_id) {
	vector<Path_Points>  t_path;
	vector<int> t_command;
	robot_path[robot_id].path_points.swap(t_path);
	robot_path[robot_id].commands.swap(t_command);
	robot_path[robot_id].all_steps = 0;
	robot_path[robot_id].cur_step = 0;
}

//��Ʒ�������ָ��ĳ�������˺�Ӧ�ô�existing_goodsȥ�� ���ⱻ�ظ�����
void remove_goods(int id) {
	auto it = find_if(existing_goods.begin(), existing_goods.end(), [id](const single_goods& p) {
		return p.id == id;
		});
	if (it != existing_goods.end()) {
		existing_goods.erase(it);
	}
}

//existing_goods��ʱ�����     1000֡�������ʧ
void update_existing_goods(int cur_zhen) {
	for (int i = 0; i < existing_goods.size(); i++) {
		if ((cur_zhen - existing_goods[i].start_zhen) > 980) {
			remove_goods(existing_goods[i].id);
		}
	}
}

//������û�л�����δ��������ʱ ��Ѱ�������     ����existing_goods��remove
single_goods search_goods(int robot_id) {
	int best_goods_flag = -1;
	//��ֵ���Ծ���  ��λ����ļ�ֵ   
	int valuable = 0;
	//int min_dis = 500;
	for (int i = 0; i < existing_goods.size(); i++) {
		int cur_distance;
		float cur_value = 0;
		cur_distance = (abs(robot[robot_id].x - existing_goods[i].x) \
			+ abs(robot[robot_id].y - existing_goods[i].y));
		if (cur_distance == 0) cur_value = 1000000;
		else cur_value = existing_goods[i].value / (cur_distance + 30);  //30Ϊ�ͷ�����  ��Ϊ����·
		if (cur_value > valuable) {
			best_goods_flag = i;
			valuable = cur_value;
			//min_dis = cur_distance;
		}
	}
	if (best_goods_flag == -1) return No_goods;
	else return existing_goods[best_goods_flag];
}

//ȡ������������λ��Ѱ  ����λ���Ͻǲ���
int search_berth(int robot_id) {
	int best_berth_id = No_dst_berth;
	int min_dis = 1000;
	for (int i = 0; i < berth_num; i++) {
		int cur_dis;
		cur_dis = abs(robot[robot_id].x - berth[i].x) + \
			abs(robot[robot_id].y - berth[i].y);
		if (min_dis > cur_dis) {
			best_berth_id = i;
			min_dis = cur_dis;
		}
	}
	return best_berth_id;
}

//ship���� ��������л����� ���� ����berthʱ�������� �� �ۿ�װж�ٶȡ��������� ����ͣ��ʱ��  ����Ϊװ������       ȥ������
int boat_search_berth(int boat_id) {
	int max_goods = 0;
	int berth_id = Berth_NoGoods;   //-1
	for (int i = 0; i < berth_num; i++) {
		if (berth[i].now_loading == 0) {          //������Ѱ���д���
			if (berth[i].goods_num > max_goods) {
				max_goods = berth[i].goods_num;
				berth_id = i;
			}
		}
	}
	if (berth_id == Berth_NoGoods)  return Berth_NoGoods;
	else {
		berth[berth_id].now_loading = 1;
		return berth_id;
	}
}

//��ʼ��
void Init()
{
	for (int i = 0; i < 200; i++)             //��ȡ��ͼ ch[0-199][0-199]
		scanf("%s", my_map[i]);
	initGraph(my_map);          //����һ��     �ɵ�ͼ��ʼ��graph
	for (int i = 0; i < berth_num; i++)     //��ȡberth��Ϣ
	{
		int id;
		scanf("%d", &id);
		berth[id].id = id;
		scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
	}
	scanf("%d", &boat_capacity);
	char okk[100];
	scanf("%s", okk);
	printf("OK\n");
	fflush(stdout);
}


int money;
int Input()          //���ص�ǰ֡��
{
	int cur_zhen;
	scanf("%d%d", &cur_zhen, &money);       //��ǰ֡��  ��ǰǮ��
	//���ϻ�����Ϣ����      
	int num;
	scanf("%d", &num);       //������Ʒ��  ����������ÿ100֡����15����Ʒ
	for (int i = 1; i <= num; i++)         //��Ʒ��λ�úͼ�ֵ
	{
		int id, x, y, val;
		scanf("%d%d%d", &x, &y, &val);
		id = goods_id;
		goods_id++;
		existing_goods.push_back({ id, x,y,val,cur_zhen });
	}
	for (int i = 0; i < robot_num; i++)       //��ȡ��������Ϣ  have_goods x,y status
	{
		scanf("%d%d%d%d", &robot[i].have_goods, &robot[i].x, &robot[i].y, &robot[i].status);
	}
	for (int i = 0; i < 5; i++)       //��ȡ������Ϣ    status  pos��Ŀ�겴λ�� -1��ʾ�������
		scanf("%d%d\n", &boat[i].status, &boat[i].pos);
	char okk[100];
	scanf("%s", okk);
	return cur_zhen;
}


int Get_Goods_Task(int i) {
	int res = No_Solution;
	do {
		single_goods dst_goods = search_goods(i);
		if (dst_goods.id == -1)  return No_Dst;        //�ȿ���������Χ����û����ѻ���   
		robot[i].dst_x = dst_goods.x;
		robot[i].dst_y = dst_goods.y;
		robot[i].dst_goods = dst_goods;
		clear_path(i);
		if (robot[i].x == dst_goods.x && robot[i].y == dst_goods.y) {
			//����Ļ�����ڽ���
			remove_goods(dst_goods.id);
			return Dst_Goods;
		}
		res = get_path(close, graph, i, robot[i].x, robot[i].y, robot[i].dst_x, robot[i].dst_y);
		//���ﲻ�ɴ� ˵���ڻ��������� ȥ��
		remove_goods(dst_goods.id);       //�ҵ����ҹ滮���� ����ҲҪȥ��
	} while (res == No_Solution);

	return Dst_Goods;
}

//stack<int> Robot_avoid_path[10];
//int search_around(int next_x, int next_y) {
//	
//}

//��ײ���� 
//�������� 
//1,�������ײ   �ַ�Ϊ����   crash_type1  A _ B       crash _type2  AB 
//2,�������  ���������˵�ĳ�������˵�һ�� ����
//     crash_type3    _ A
//                    B
// type1 �� type3����  Bͣ����   ����wait  ��type3����� type1�����type2
// type2   ��back  
// 3, A������ 
//  

//������0  �ò����ȼ���� Ҫ�����л������ò� ����
int crash_detec(int robot_id) {
	int input_cur_step = robot_path[robot_id].cur_step;
	int input_all_steps = robot_path[robot_id].all_steps;
	Path_Points input_nowp;
	Path_Points input_nextp;
	input_nowp = robot_path[robot_id].path_points[input_cur_step];      //robot_input�ĵ�ǰ��
	input_nextp = robot_path[robot_id].path_points[input_cur_step + 1];      //robot_input����һ����
	for (int i = 0; i < robot_num; i++) {  //���ϻ����˵ļ��   ����������˵ļ��
		if (i == robot_id) continue;
		if (robot[i].status == 0 && robot[i].x == input_nowp.x && robot[i].y == input_nowp.y) {  //�����˹���
			return Meet_Bad;
		}
		if (robot_path[i].all_steps == 0 && robot_path[i].cur_step == 0\
			&& robot[i].x == input_nowp.x && robot[i].y == input_nowp.y) {
			return Meet_Bad;          //������all_steps==0 cur_step==0 ˵���û�����û����
		}
	}
	for (int i = robot_id + 1; i < robot_num; i++) {         //����������ײ
		//printf("-------------------------------------------------------------------------------in for----\n");
		if (robot_path[i].all_steps == 0 && robot_path[i].cur_step == 0)  continue;
		int i_cur_step = robot_path[i].cur_step;
		int i_all_steps = robot_path[i].all_steps;
		Path_Points i_nextp;
		Path_Points i_nowp;
		i_nextp = robot_path[i].path_points[i_cur_step + 1];            //robot_i ����һ����
		i_nowp = robot_path[i].path_points[i_cur_step];            //robot_i �ĵ�ǰ��
		if (i_cur_step == i_all_steps) {            //������i�Ѿ����յ���
			return No_Crash;
		}
		if (i_nextp.x == input_nextp.x && i_nextp.y == input_nextp.y\
			&& (i_nowp.x != input_nowp.x || i_nowp.y != input_nowp.y)) {        //�ж��Ƿ����ײ   type1  3
			//printf("-------------------3------------------------------------------------------------\n");
			return Robot_wait;
		}
		if (i_nextp.x == input_nowp.x && i_nextp.y == input_nowp.y\
			&& i_nowp.x == input_nextp.x && i_nowp.y == input_nextp.y) {  //type2    ��2������� ���input_robotû������ 
			return Crash_type2;
			//	printf("------------------2-----------------------------------------------------------\n");
		}
	}
	return No_Crash;
}

//������i���¹滮·��
int Re_Plan(int i, int dst_type) {
	//�����µ�ͼ
	for (int i = 0; i < Read_Size; i++) {
		for (int j = 0; j < Read_Size; j++)
			temp_map[i][j] = my_map[i][j];
	}
	int cur_step = robot_path[i].cur_step;
	int next_step = cur_step + 1;
	int next_x = robot_path[i].path_points[next_step].x;
	int next_y = robot_path[i].path_points[next_step].y;
	temp_map[next_x][next_y] = '#';     //��һ����Ϊ�ϰ���
	init_tempGraph(temp_map);      //��ʼ����ʱgraph
	clear_path(i);
	int res = get_path(temp_cls, temp_g, i, robot[i].x, robot[i].y, robot[i].dst_x, robot[i].dst_y);
	if (res == No_Solution) return No_Solution;
	else return dst_type;
}

int Get_Berth_Task(int i) {
	int dst_id = search_berth(i);
	robot[i].dst_berth_id = dst_id;
	if (dst_id == -1) return No_dst_berth;
	else {
		robot[i].dst_x = berth[dst_id].x + rand() % 4;
		robot[i].dst_y = berth[dst_id].y + rand() % 4;    //Ŀ�ĵ�ȡ��� ??? 	
		clear_path(i);
		get_path(close, graph, i, robot[i].x, robot[i].y, robot[i].dst_x, robot[i].dst_y);
		return Find_dst_berth;
	}
}

int all_goods_num = 0;
int final_money = 0;
int robot_money = 0;

struct attribute {
	int state_zhen;
	int num_goods;
	int value_goods;
};

struct Berth_State {
	int berth_id;
	int current_num_goods=0;
	int current_value_goods=0;
	vector <attribute> berth_attri = {};
}berth_state[berth_num];

#include <fstream>

int main()
{	
	
	for (int index = 0; index < berth_num; index++) {
		berth_state[index].berth_id = index;
	}
	Init();   //��ȡmap berth
	for (int zhen = 1; zhen <= 15000; zhen++)
	{
		int id = Input();  //  id Ϊ��ǰʵ��֡��
		update_existing_goods(id);         //���³��ϻ���  �������ʧ��
		//printf("goods nums: % d\n", existing_goods.size());
		//ÿ�������˵Ŀ���
		for (int i = 0; i < 10; i++) {

			//�쳣״̬����
			// 1�������˴��ڹ���״̬����ײ�ˣ�                                  
			if (robot[i].status == 0) {
				if (robot[i].dst_type == Dst_Goods && robot[i].have_goods != 1) { //�ѹ滮��δȡ�Ļ���Ҫ�Żص�����
					existing_goods.push_back(robot[i].dst_goods);
				}
				robot[i].dst_type = No_Dst;
				clear_path(i);
				continue;
			}
			//2, To_Berth״̬�� have_goods=0  
			if ((robot[i].dst_type == Dst_Berth) && (robot[i].have_goods == 0)) {
				robot[i].dst_type = No_Dst;
			}

			//�ƶ�ǰ �������ڸۿ�Ŀ�ĵ����л���  ���»���
			if ((my_map[robot[i].x][robot[i].y] == 'B') && (robot[i].have_goods == 1)) {
				printf("pull %d\n", i);
				clear_path(i);
				all_goods_num++;
				final_money = final_money + robot[i].dst_goods.value;          //��¼Ǯ��  
				berth[robot[i].dst_berth_id].goods_num++;           //�ۿڻ�����++                  
				berth_state[robot[i].dst_berth_id].current_num_goods++;
				berth_state[robot[i].dst_berth_id].current_value_goods += robot[i].dst_goods.value;
				berth_state[robot[i].dst_berth_id].berth_attri.push_back({id, berth_state[robot[i].dst_berth_id].current_num_goods,berth_state[robot[i].dst_berth_id].current_value_goods });
				robot[i].have_goods = 0;
				robot[i].dst_type = No_Dst;
			}
			// �ƶ�ǰ �������ڻ��ﴦ��Ŀǰ�޻��� ��ȡ��   
			if ((my_map[robot[i].x][robot[i].y] != 'B') && (robot[i].x == robot[i].dst_x) && (robot[i].y == robot[i].dst_y) \
				&& (robot[i].have_goods == 0) && robot[i].dst_type == Dst_Goods) {
				printf("get %d\n", i);
				robot_money = robot_money + robot[i].dst_goods.value;
				clear_path(i);
				robot[i].have_goods = 1;
				robot[i].dst_type = No_Dst;
			}
			//�������������޻���ʱ ��Ѱ�����Ʒ ����·���滮  ����To_Goods״̬          
			if ((robot[i].dst_type == No_Dst) && (robot[i].have_goods == 0)) {
				if (existing_goods.size() == 0) continue;   //����û����
				int res;
				res = Get_Goods_Task(i);
				if (res == No_Dst) continue;
				else robot[i].dst_type = Dst_Goods;
			}
			//������������ʱ�л���  ��Ѱ���berth ����·���滮  ����To_Berth״̬
			if ((robot[i].dst_type == No_Dst) && (robot[i].have_goods == 1)) {
				int res = Get_Berth_Task(i);
				if (res == Find_dst_berth) robot[i].dst_type = Dst_Berth;
			}
			//Dst_Goods״̬ ����Dst_Berth  ��move ������      
			if (robot[i].dst_type == Dst_Goods || robot[i].dst_type == Dst_Berth) {
				if (0 <= robot_path[i].cur_step && robot_path[i].cur_step < robot_path[i].all_steps\
					&& robot_path[i].all_steps != 0)                    //����Ƿ񳬳��ܲ���
				{
					int detec_res = crash_detec(i);         		//��ײ���  
					if (detec_res == Robot_wait) {            //���Ϊwait ����ָ��
						continue;
					}
					if (detec_res == Meet_Bad) {  //�������ϻ�����  ���¹滮·��
						//cout << "--------------------------------------------meet_bad_replan----" << endl;
						int res = Re_Plan(i, robot[i].dst_type);
						if (res == No_Solution) continue;      //�Ҳ���·����ȴ�
						else detec_res = No_Crash;      //�ҵ�·������·����
					}
					if (detec_res == Crash_type2) {      //����2��ȡ���˲���
						if (0 < robot_path[i].cur_step && robot_path[i].cur_step < robot_path[i].all_steps) {//����       Ҫ��֤cur_step���ǵ�һ�����ܻ���
							robot_path[i].cur_step--;
							int command = robot_path[i].commands[robot_path[i].cur_step];  //command0-3
							if (command == 0 || command == 2) command++;   //0->1  2->3   
							else command--;//  1->0   3->2
							printf("move %d %d\n", i, command);
							continue;
						}
						else {            //cur_step�ǵ�һ��  û�������� ���¹滮·��
							//cout << "--------------------------------------------can_back_replan----" << endl;
							int res = Re_Plan(i, robot[i].dst_type);
							if (res == No_Solution) continue;      //�Ҳ���·����ȴ�
							else detec_res = No_Crash;      //�ҵ�·������·����
						}
					}
					//     No_Crash  ������
					if (detec_res == No_Crash) {
						int command = robot_path[i].commands[robot_path[i].cur_step];
						printf("move %d %d\n", i, command);
						robot_path[i].cur_step++;
					}
				}
			}
			//�ƶ��� �������ڸۿ�Ŀ�ĵ����л���  ���»���
			//    �������ڸۿ� ���߻�����move��Ҫ�ڸۿ�
			if ((my_map[robot[i].x][robot[i].y] == 'B' && robot[i].have_goods == 1 && robot[i].dst_type == Dst_Berth)\
				|| (robot_path[i].cur_step == robot_path[i].all_steps && robot[i].dst_type == Dst_Berth && robot_path[i].all_steps != 0 && robot[i].have_goods == 1))
			{
				printf("pull %d\n", i);
				all_goods_num++;
				final_money = final_money + robot[i].dst_goods.value;
				clear_path(i);
				berth[robot[i].dst_berth_id].goods_num++;           //�ۿڻ�����++
				berth_state[robot[i].dst_berth_id].current_num_goods++;
				berth_state[robot[i].dst_berth_id].current_value_goods += robot[i].dst_goods.value;
				berth_state[robot[i].dst_berth_id].berth_attri.push_back({ id, berth_state[robot[i].dst_berth_id].current_num_goods,berth_state[robot[i].dst_berth_id].current_value_goods });
				robot[i].dst_type = No_Dst;         //���º�����Ŀ��                   
				robot[i].have_goods = 0;
			}
			// �ƶ��� �������ڻ��ﴦ��Ŀǰ�޻��� ��ȡ��   
			// ���߻�����move�󵽻���  
			if ((robot[i].dst_type == Dst_Goods && robot[i].x == robot[i].dst_x && robot[i].y == robot[i].dst_y && robot[i].have_goods == 0)\
				|| (robot[i].dst_type == Dst_Goods && robot_path[i].cur_step == robot_path[i].all_steps && robot_path[i].all_steps != 0 && robot[i].have_goods == 0))
			{
				printf("get %d\n", i);
				robot_money = robot_money + robot[i].dst_goods.value;
				clear_path(i);
				robot[i].dst_type = No_Dst;
				robot[i].have_goods = 1;
			}
		}
		//���Ŀ���
		
		for (int i = 0; i < 5; i++) {
			if (boat[i].status == 1 && boat[i].pos != -1 && (14990 - id) <= berth[boat[i].pos].transport_time) {   //��������ʱ��  ���ʱ�䲻���� �뿪
				printf("go %d\n", i);
				continue;
			}
			if (boat[i].status == 0)  continue;            //�����о��Ȳ���
			//int goods_num;  //�������л�������
			//int pos;       // ��ʱ�����ڵ�λ�ã������Ϊ-1�����಴λΪ��Ӧ���
			//int status;   //��ʾ�Ƿ񵽴��������߲���, ���������status��ֵΪ0��ͣ����status��ֵΪ1,�ڲ�λ��ȴ���ֵΪ2
			//int task_berth = -1;  //��ʼ״̬��task_berth��ʾ����Ӧ�Ĳ�λ*
			//int stay_start;       //���ﲴ��ʱ��ʵ��֡��*
			//int arrive_flag = -1; //���ﲴ���־*
			if (i == 0) {
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth == -1) {  //��Ϊ��ʼ��״̬
					printf("ship %d 2\n", i);  //ȥ berth2
					boat[i].task_berth = 2;
					boat[i].task_next_berth = 0;  //���ɲ�λ2ȥ��λ0
					continue;
				}
				if (boat[i].status == 1 && boat[i].pos != -1 && boat[i].arrive_flag == -1) {         //�յ���ʱ���¼��ʼ֡
					boat[i].arrive_flag = 1;
					boat[i].stay_start = id;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_berth) {
					//int load_goods_num = min(berth[boat[i].task_berth].goods_num, berth[boat[i].task_berth].loading_speed * (id - boat[i].stay_start));
					//boat[i].goods_num += load_goods_num;
					//berth[boat[i].task_berth].goods_num -= load_goods_num;
					//boat[i].stay_start = id;
					if (id + berth[boat[i].task_berth].transport_time > 14985 || id + berth[boat[i].task_next_berth].transport_time > 14485) {
						printf("go %d\n", i);
						continue;
					}
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_berth && 
					((id - boat[i].stay_start) * berth[boat[i].task_berth].loading_speed >= boat_capacity)) {
					printf("ship %d 0\n", i);
					boat[i].arrive_flag = -1;
					continue;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_next_berth && (id + berth[boat[i].task_next_berth].transport_time) > 14985) {
					printf("go %d\n", i);
					continue;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_next_berth && 
					(id - boat[i].stay_start) * berth[boat[i].task_next_berth].loading_speed >= boat_capacity) {
					printf("go %d\n", i);
					//berth[boat[i].task_berth].goods_num -= (boat_capacity - boat[i].goods_num);
					boat[i].arrive_flag = -1;
					//boat[i].goods_num = 0;
					continue;
				}
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth != -1) {   //����ĳ�����㵽������� 
					printf("ship %d 2\n", i);
					continue;
				}
			}
			if (i == 1) {
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth == -1) {  //��Ϊ��ʼ��״̬
					printf("ship %d 7\n", i);  //ȥ berth7
					boat[i].task_berth = 7;
					boat[i].task_next_berth = 8;  //���ɲ�λ7ȥ��λ8
					continue;
				}
				if (boat[i].status == 1 && boat[i].pos != -1 && boat[i].arrive_flag == -1) {         //�յ���ʱ���¼��ʼ֡
					boat[i].arrive_flag = 1;
					boat[i].stay_start = id;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_berth) {
					//int load_goods_num = min(berth[boat[i].task_berth].goods_num, berth[boat[i].task_berth].loading_speed * (id - boat[i].stay_start));
					//boat[i].goods_num += load_goods_num;
					//berth[boat[i].task_berth].goods_num -= load_goods_num;
					//boat[i].stay_start = id;
					if (id + berth[boat[i].task_berth].transport_time > 14985 || id + berth[boat[i].task_next_berth].transport_time > 14485) {
						printf("go %d\n", i);
						continue;
					}
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_berth &&
					(id - boat[i].stay_start) * berth[boat[i].task_berth].loading_speed >= boat_capacity) {
					printf("ship %d 8\n", i);
					boat[i].arrive_flag = -1;
					continue;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_next_berth && (id + berth[boat[i].task_next_berth].transport_time) > 14985) {
					printf("go %d\n", i);
					continue;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_next_berth &&
					(id - boat[i].stay_start) * berth[boat[i].task_next_berth].loading_speed >= boat_capacity) {
					printf("go %d\n", i);
					//berth[boat[i].task_berth].goods_num -= (boat_capacity - boat[i].goods_num);
					boat[i].arrive_flag = -1;
					//boat[i].goods_num = 0;
					continue;
				}
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth != -1) {   //����ĳ�����㵽������� 
					printf("ship %d 7\n", i);
					continue;
				}
			}
			if (i == 2) {
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth == -1) {  //��Ϊ��ʼ��״̬
					printf("ship %d 4\n", i);  //ȥ berth6
					boat[i].task_berth = 4;
					boat[i].task_next_berth = 9;  //���ɲ�λ6ȥ��λ3
					continue;
				}
				if (boat[i].status == 1 && boat[i].pos != -1 && boat[i].arrive_flag == -1) {         //�յ���ʱ���¼��ʼ֡
					boat[i].arrive_flag = 1;
					boat[i].stay_start = id;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_berth) {
					//int load_goods_num = min(berth[boat[i].task_berth].goods_num, berth[boat[i].task_berth].loading_speed * (id - boat[i].stay_start));
					//boat[i].goods_num += load_goods_num;
					//berth[boat[i].task_berth].goods_num -= load_goods_num;
					//boat[i].stay_start = id;
					if (id + berth[boat[i].task_berth].transport_time > 14985 || id + berth[boat[i].task_next_berth].transport_time > 14485) {
						printf("go %d\n", i);
						continue;
					}
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_berth &&
					(id - boat[i].stay_start) * berth[boat[i].task_berth].loading_speed >= boat_capacity) {
					printf("ship %d 9\n", i);
					boat[i].arrive_flag = -1;
					continue;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_next_berth && (id + berth[boat[i].task_next_berth].transport_time) > 14985) {
					printf("go %d\n", i);
					continue;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_next_berth &&
					(id - boat[i].stay_start) * berth[boat[i].task_next_berth].loading_speed >= boat_capacity) {
					printf("go %d\n", i);
					//berth[boat[i].task_berth].goods_num -= (boat_capacity - boat[i].goods_num);
					boat[i].arrive_flag = -1;
					//boat[i].goods_num = 0;
					continue;
				}
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth != -1) {   //����ĳ�����㵽������� 
					printf("ship %d 4\n", i);
					continue;
				}
			}
			if (i == 3) {
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth == -1) {  //��Ϊ��ʼ��״̬
					printf("ship %d 6\n", i);  //ȥ berth8
					boat[i].task_berth = 6;
					boat[i].task_next_berth = 1;  //���ɲ�λ8ȥ��λ5
					continue;
				}
				if (boat[i].status == 1 && boat[i].pos != -1 && boat[i].arrive_flag == -1) {         //�յ���ʱ���¼��ʼ֡
					boat[i].arrive_flag = 1;
					boat[i].stay_start = id;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_berth) {
					//int load_goods_num = min(berth[boat[i].task_berth].goods_num, berth[boat[i].task_berth].loading_speed * (id - boat[i].stay_start));
					//boat[i].goods_num += load_goods_num;
					//berth[boat[i].task_berth].goods_num -= load_goods_num;
					//boat[i].stay_start = id;
					if (id + berth[boat[i].task_berth].transport_time > 14985 || id + berth[boat[i].task_next_berth].transport_time > 14485) {
						printf("go %d\n", i);
						continue;
					}
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_berth &&
					(id - boat[i].stay_start) * berth[boat[i].task_berth].loading_speed >= boat_capacity) {
					printf("ship %d 1\n", i);
					boat[i].arrive_flag = -1;
					continue;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_next_berth && (id + berth[boat[i].task_next_berth].transport_time) > 14985) {
					printf("go %d\n", i);
					continue;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_next_berth &&
					(id - boat[i].stay_start) * berth[boat[i].task_next_berth].loading_speed >= boat_capacity) {
					printf("go %d\n", i);
					//berth[boat[i].task_berth].goods_num -= (boat_capacity - boat[i].goods_num);
					boat[i].arrive_flag = -1;
					//boat[i].goods_num = 0;
					continue;
				}
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth != -1) {   //����ĳ�����㵽������� 
					printf("ship %d 6\n", i);
					continue;
				}
			}
			if (i == 4) {
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth == -1) {  //��Ϊ��ʼ��״̬
					printf("ship %d 5\n", i);  //ȥ berth9
					boat[i].task_berth = 5;
					boat[i].task_next_berth = 3;  //���ɲ�λ9ȥ��λ7
					continue;
				}
				if (boat[i].status == 1 && boat[i].pos != -1 && boat[i].arrive_flag == -1) {         //�յ���ʱ���¼��ʼ֡
					boat[i].arrive_flag = 1;
					boat[i].stay_start = id;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_berth) {
					//int load_goods_num = min(berth[boat[i].task_berth].goods_num, berth[boat[i].task_berth].loading_speed * (id - boat[i].stay_start));
					//boat[i].goods_num += load_goods_num;
					//berth[boat[i].task_berth].goods_num -= load_goods_num;
					//boat[i].stay_start = id;
					if (id + berth[boat[i].task_berth].transport_time > 14985 || id + berth[boat[i].task_next_berth].transport_time > 14485) {
						printf("go %d\n", i);
						continue;
					}
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_berth &&
					(id - boat[i].stay_start) * berth[boat[i].task_berth].loading_speed >= boat_capacity) {
					printf("ship %d 3\n", i);
					boat[i].arrive_flag = -1;
					continue;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_next_berth && (id + berth[boat[i].task_next_berth].transport_time) > 14985) {
					printf("go %d\n", i);
					continue;
				}
				if (boat[i].arrive_flag == 1 && boat[i].pos == boat[i].task_next_berth &&
					(id - boat[i].stay_start) * berth[boat[i].task_next_berth].loading_speed >= boat_capacity) {
					printf("go %d\n", i);
					//berth[boat[i].task_berth].goods_num -= (boat_capacity - boat[i].goods_num);
					boat[i].arrive_flag = -1;
					//boat[i].goods_num = 0;
					continue;
				}
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth != -1) {   //����ĳ�����㵽������� 
					printf("ship %d 5\n", i);
					continue;
				}
			}

			//if (boat[i].status == 1 && boat[i].pos != -1 && (14990 - id) <= berth[boat[i].pos].transport_time) {   //��������ʱ��  ���ʱ�䲻���� �뿪
			//	printf("go %d\n", i);
			//	continue;
			//}
			////��������� ship��һ��û��ȥ�һ����Ĳ�λ
			//if (boat[i].status == 1 && boat[i].pos == -1) {
			//	boat[i].goods_num = 0;     //������0
			//	int berth_id = boat_search_berth(i);
			//	if (berth_id != Berth_NoGoods) {
			//		printf("ship %d %d\n", i, berth_id);
			//		continue;
			//	}
			//}
			////������ȥ�����        go֮�󲻽�����Ʒ������Ϣ 
			//if ((boat[i].pos != -1) && (boat[i].status == 1) && (boat_capacity == boat[i].goods_num)) {
			//	berth[boat[i].pos].now_loading = 0;
			//	printf("go %d\n", i);
			//	continue;
			//}
			////	 �����ڵ�berthû������ ȥ������λ
			//if ((boat[i].pos != -1) && (berth[boat[i].pos].goods_num == 0)) {
			//	berth[boat[i].pos].now_loading = 0;
			//	int berth_id = boat_search_berth(i);
			//	if (berth_id != Berth_NoGoods) {
			//		printf("ship %d %d\n", i, berth_id);
			//		continue;
			//	}
			//}
			////���ڲ�λ��   ���´���berth����Ʒ��
			//if ((boat[i].pos != -1) && (boat[i].status == 1)) {
			//	if (berth[boat[i].pos].goods_num > berth[boat[i].pos].loading_speed) {       //������װһ�� 
			//		// װһ�λ᲻�����
			//		if (boat_capacity < berth[boat[i].pos].loading_speed) {
			//			boat[i].goods_num += berth[boat[i].pos].loading_speed;
			//			berth[boat[i].pos].goods_num -= berth[boat[i].pos].loading_speed;
			//		}
			//		else {        //�����ʱ
			//			berth[boat[i].pos].goods_num = berth[boat[i].pos].goods_num - (boat_capacity - boat[i].goods_num);
			//			boat[i].goods_num = boat_capacity;
			//		}
			//	}
			//	else {  // ����װһ��
			//		boat[i].goods_num += berth[boat[i].pos].goods_num;
			//		berth[boat[i].pos].goods_num = 0;
			//	}
			//}
		}
		zhen = id;   //֡�Ÿ���Ϊ��ǰʵ��֡��
		puts("OK");
		fflush(stdout);

	}

	fstream outfile;
	outfile.open("data_berth.txt", ios::app);

	for (Berth_State bs : berth_state) {
		for (attribute sa : bs.berth_attri) {
			outfile << bs.berth_id << "\t" << sa.state_zhen << "\t" << sa.num_goods << "\t" << sa.value_goods << endl;
		}
	}
	outfile.close();
	return 0;
}