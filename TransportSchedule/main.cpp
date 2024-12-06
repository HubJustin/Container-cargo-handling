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
//货物坐标存储  判题器期望每100帧生成15个物品
struct single_goods {
	int id;          //id是唯一的  用于物品管理
	int x, y, value;
	int start_zhen;
};
struct single_goods No_goods = { -1 };
vector<single_goods> existing_goods = {};

//机器人 泊位及船的定义
const int robot_num = 10;
const int berth_num = 10;

struct Robot
{
	int have_goods, x, y, status;
	int dst_x;
	int	dst_y;   //终点
	int dst_berth_id;
	int dst_type = No_Dst;
	single_goods dst_goods = No_goods;
	Robot() {}
	Robot(int startX, int startY) {
		x = startX;
		y = startY;
	}
}robot[robot_num];  //10个机器人 0-9

struct Berth
{
	int id;
	int x;      //左上角坐标
	int y;
	int transport_time;
	int loading_speed;
	int now_loading = 0;         //0表示没有船 1表示有船在装卸中 或者有船过去了
	int goods_num;  //泊位现有物品数   
	Berth() {}
	Berth(int x, int y, int transport_time, int loading_speed) {
		this->x = x;
		this->y = y;
		this->transport_time = transport_time;
		this->loading_speed = loading_speed;
	}
}berth[berth_num];  //10个泊位 0-9

int boat_capacity;
struct Boat
{
	int goods_num;  //船上已有货物数量
	int pos, status;
	int task_berth = -1;  //初始状态
	int task_next_berth = -1;
	int stay_start;
	int arrive_flag = -1;
}boat[5];   //共五艘船  0-4


//机器人路径存储方法
struct Path_Points {
	int x, y;
};
struct Robot_Path {
	vector<Path_Points>  path_points;      //路径点数比指令数多1  
	vector<int> commands;   //指令           非递归版本指令和路径反着存放
	int all_steps;        //总步数
	int cur_step = 0;   //当前步数  起点记为0
}robot_path[10];      //10个二维数组对应10个机器人的路径 {{x,y,command},....} 

//递归
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

//到达终点后路径清空  机器人状态回归原始
void clear_path(int robot_id) {
	vector<Path_Points>  t_path;
	vector<int> t_command;
	robot_path[robot_id].path_points.swap(t_path);
	robot_path[robot_id].commands.swap(t_command);
	robot_path[robot_id].all_steps = 0;
	robot_path[robot_id].cur_step = 0;
}

//物品被分配给指定某个机器人后，应该从existing_goods去除 避免被重复分配
void remove_goods(int id) {
	auto it = find_if(existing_goods.begin(), existing_goods.end(), [id](const single_goods& p) {
		return p.id == id;
		});
	if (it != existing_goods.end()) {
		existing_goods.erase(it);
	}
}

//existing_goods随时间更新     1000帧后货物消失
void update_existing_goods(int cur_zhen) {
	for (int i = 0; i < existing_goods.size(); i++) {
		if ((cur_zhen - existing_goods[i].start_zhen) > 980) {
			remove_goods(existing_goods[i].id);
		}
	}
}

//机器人没有货物且未分配任务时 搜寻最近货物     并从existing_goods中remove
single_goods search_goods(int robot_id) {
	int best_goods_flag = -1;
	//价值除以距离  单位距离的价值   
	int valuable = 0;
	//int min_dis = 500;
	for (int i = 0; i < existing_goods.size(); i++) {
		int cur_distance;
		float cur_value = 0;
		cur_distance = (abs(robot[robot_id].x - existing_goods[i].x) \
			+ abs(robot[robot_id].y - existing_goods[i].y));
		if (cur_distance == 0) cur_value = 1000000;
		else cur_value = existing_goods[i].value / (cur_distance + 30);  //30为惩罚因子  因为会绕路
		if (cur_value > valuable) {
			best_goods_flag = i;
			valuable = cur_value;
			//min_dis = cur_distance;
		}
	}
	if (best_goods_flag == -1) return No_goods;
	else return existing_goods[best_goods_flag];
}

//取货后进行最近泊位搜寻  按泊位左上角查找
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

//ship运输 需计算已有货物数 根据 到达berth时货物数量 及 港口装卸速度、船的容量 计算停靠时间  策略为装满再走       去货最多的
int boat_search_berth(int boat_id) {
	int max_goods = 0;
	int berth_id = Berth_NoGoods;   //-1
	for (int i = 0; i < berth_num; i++) {
		if (berth[i].now_loading == 0) {          //不能搜寻已有船的
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

//初始化
void Init()
{
	for (int i = 0; i < 200; i++)             //读取地图 ch[0-199][0-199]
		scanf("%s", my_map[i]);
	initGraph(my_map);          //仅需一次     由地图初始化graph
	for (int i = 0; i < berth_num; i++)     //读取berth信息
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
int Input()          //返回当前帧号
{
	int cur_zhen;
	scanf("%d%d", &cur_zhen, &money);       //当前帧号  当前钱数
	//场上货物信息更新      
	int num;
	scanf("%d", &num);       //新增物品数  判题器期望每100帧生成15个物品
	for (int i = 1; i <= num; i++)         //物品的位置和价值
	{
		int id, x, y, val;
		scanf("%d%d%d", &x, &y, &val);
		id = goods_id;
		goods_id++;
		existing_goods.push_back({ id, x,y,val,cur_zhen });
	}
	for (int i = 0; i < robot_num; i++)       //读取机器人信息  have_goods x,y status
	{
		scanf("%d%d%d%d", &robot[i].have_goods, &robot[i].x, &robot[i].y, &robot[i].status);
	}
	for (int i = 0; i < 5; i++)       //读取船的信息    status  pos（目标泊位） -1表示在虚拟点
		scanf("%d%d\n", &boat[i].status, &boat[i].pos);
	char okk[100];
	scanf("%s", okk);
	return cur_zhen;
}


int Get_Goods_Task(int i) {
	int res = No_Solution;
	do {
		single_goods dst_goods = search_goods(i);
		if (dst_goods.id == -1)  return No_Dst;        //先看可搜索范围内有没有最佳货物   
		robot[i].dst_x = dst_goods.x;
		robot[i].dst_y = dst_goods.y;
		robot[i].dst_goods = dst_goods;
		clear_path(i);
		if (robot[i].x == dst_goods.x && robot[i].y == dst_goods.y) {
			//最近的货物就在脚下
			remove_goods(dst_goods.id);
			return Dst_Goods;
		}
		res = get_path(close, graph, i, robot[i].x, robot[i].y, robot[i].dst_x, robot[i].dst_y);
		//货物不可达 说明在货物在死区 去除
		remove_goods(dst_goods.id);       //找到了且规划好了 货物也要去除
	} while (res == No_Solution);

	return Dst_Goods;
}

//stack<int> Robot_avoid_path[10];
//int search_around(int next_x, int next_y) {
//	
//}

//碰撞处理 
//几种类型 
//1,相对型碰撞   又分为两种   crash_type1  A _ B       crash _type2  AB 
//2,非相对型  两个机器人的某个机器人等一下 即可
//     crash_type3    _ A
//                    B
// type1 和 type3采用  B停下来   返回wait  则type3解决了 type1变成了type2
// type2   则back  
// 3, A故障了 
//  

//机器人0  让步优先级最高 要给所有机器人让步 类推
int crash_detec(int robot_id) {
	int input_cur_step = robot_path[robot_id].cur_step;
	int input_all_steps = robot_path[robot_id].all_steps;
	Path_Points input_nowp;
	Path_Points input_nextp;
	input_nowp = robot_path[robot_id].path_points[input_cur_step];      //robot_input的当前点
	input_nextp = robot_path[robot_id].path_points[input_cur_step + 1];      //robot_input的下一个点
	for (int i = 0; i < robot_num; i++) {  //故障机器人的检测   无任务机器人的检测
		if (i == robot_id) continue;
		if (robot[i].status == 0 && robot[i].x == input_nowp.x && robot[i].y == input_nowp.y) {  //机器人故障
			return Meet_Bad;
		}
		if (robot_path[i].all_steps == 0 && robot_path[i].cur_step == 0\
			&& robot[i].x == input_nowp.x && robot[i].y == input_nowp.y) {
			return Meet_Bad;          //机器人all_steps==0 cur_step==0 说明该机器人没任务
		}
	}
	for (int i = robot_id + 1; i < robot_num; i++) {         //其他类型碰撞
		//printf("-------------------------------------------------------------------------------in for----\n");
		if (robot_path[i].all_steps == 0 && robot_path[i].cur_step == 0)  continue;
		int i_cur_step = robot_path[i].cur_step;
		int i_all_steps = robot_path[i].all_steps;
		Path_Points i_nextp;
		Path_Points i_nowp;
		i_nextp = robot_path[i].path_points[i_cur_step + 1];            //robot_i 的下一个点
		i_nowp = robot_path[i].path_points[i_cur_step];            //robot_i 的当前点
		if (i_cur_step == i_all_steps) {            //机器人i已经在终点了
			return No_Crash;
		}
		if (i_nextp.x == input_nextp.x && i_nextp.y == input_nextp.y\
			&& (i_nowp.x != input_nowp.x || i_nowp.y != input_nowp.y)) {        //判断是否会相撞   type1  3
			//printf("-------------------3------------------------------------------------------------\n");
			return Robot_wait;
		}
		if (i_nextp.x == input_nowp.x && i_nextp.y == input_nowp.y\
			&& i_nowp.x == input_nextp.x && i_nowp.y == input_nextp.y) {  //type2    在2的情况下 如果input_robot没法回退 
			return Crash_type2;
			//	printf("------------------2-----------------------------------------------------------\n");
		}
	}
	return No_Crash;
}

//机器人i重新规划路径
int Re_Plan(int i, int dst_type) {
	//构造新地图
	for (int i = 0; i < Read_Size; i++) {
		for (int j = 0; j < Read_Size; j++)
			temp_map[i][j] = my_map[i][j];
	}
	int cur_step = robot_path[i].cur_step;
	int next_step = cur_step + 1;
	int next_x = robot_path[i].path_points[next_step].x;
	int next_y = robot_path[i].path_points[next_step].y;
	temp_map[next_x][next_y] = '#';     //下一点标记为障碍物
	init_tempGraph(temp_map);      //初始化临时graph
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
		robot[i].dst_y = berth[dst_id].y + rand() % 4;    //目的地取随机 ??? 	
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
	Init();   //读取map berth
	for (int zhen = 1; zhen <= 15000; zhen++)
	{
		int id = Input();  //  id 为当前实际帧号
		update_existing_goods(id);         //更新场上货物  处理掉消失的
		//printf("goods nums: % d\n", existing_goods.size());
		//每个机器人的控制
		for (int i = 0; i < 10; i++) {

			//异常状态更新
			// 1，机器人处于故障状态（碰撞了）                                  
			if (robot[i].status == 0) {
				if (robot[i].dst_type == Dst_Goods && robot[i].have_goods != 1) { //已规划但未取的货物要放回到场上
					existing_goods.push_back(robot[i].dst_goods);
				}
				robot[i].dst_type = No_Dst;
				clear_path(i);
				continue;
			}
			//2, To_Berth状态下 have_goods=0  
			if ((robot[i].dst_type == Dst_Berth) && (robot[i].have_goods == 0)) {
				robot[i].dst_type = No_Dst;
			}

			//移动前 机器人在港口目的点且有货物  放下货物
			if ((my_map[robot[i].x][robot[i].y] == 'B') && (robot[i].have_goods == 1)) {
				printf("pull %d\n", i);
				clear_path(i);
				all_goods_num++;
				final_money = final_money + robot[i].dst_goods.value;          //记录钱数  
				berth[robot[i].dst_berth_id].goods_num++;           //港口货物数++                  
				berth_state[robot[i].dst_berth_id].current_num_goods++;
				berth_state[robot[i].dst_berth_id].current_value_goods += robot[i].dst_goods.value;
				berth_state[robot[i].dst_berth_id].berth_attri.push_back({id, berth_state[robot[i].dst_berth_id].current_num_goods,berth_state[robot[i].dst_berth_id].current_value_goods });
				robot[i].have_goods = 0;
				robot[i].dst_type = No_Dst;
			}
			// 移动前 机器人在货物处且目前无货物 则取货   
			if ((my_map[robot[i].x][robot[i].y] != 'B') && (robot[i].x == robot[i].dst_x) && (robot[i].y == robot[i].dst_y) \
				&& (robot[i].have_goods == 0) && robot[i].dst_type == Dst_Goods) {
				printf("get %d\n", i);
				robot_money = robot_money + robot[i].dst_goods.value;
				clear_path(i);
				robot[i].have_goods = 1;
				robot[i].dst_type = No_Dst;
			}
			//机器人无任务无货物时 搜寻最近物品 进行路径规划  进入To_Goods状态          
			if ((robot[i].dst_type == No_Dst) && (robot[i].have_goods == 0)) {
				if (existing_goods.size() == 0) continue;   //场上没货物
				int res;
				res = Get_Goods_Task(i);
				if (res == No_Dst) continue;
				else robot[i].dst_type = Dst_Goods;
			}
			//机器人无任务时有货物  搜寻最近berth 进行路径规划  进入To_Berth状态
			if ((robot[i].dst_type == No_Dst) && (robot[i].have_goods == 1)) {
				int res = Get_Berth_Task(i);
				if (res == Find_dst_berth) robot[i].dst_type = Dst_Berth;
			}
			//Dst_Goods状态 或者Dst_Berth  则move 并更新      
			if (robot[i].dst_type == Dst_Goods || robot[i].dst_type == Dst_Berth) {
				if (0 <= robot_path[i].cur_step && robot_path[i].cur_step < robot_path[i].all_steps\
					&& robot_path[i].all_steps != 0)                    //检测是否超出总步数
				{
					int detec_res = crash_detec(i);         		//碰撞检测  
					if (detec_res == Robot_wait) {            //结果为wait 则无指令
						continue;
					}
					if (detec_res == Meet_Bad) {  //遇到故障机器人  重新规划路径
						//cout << "--------------------------------------------meet_bad_replan----" << endl;
						int res = Re_Plan(i, robot[i].dst_type);
						if (res == No_Solution) continue;      //找不到路径则等待
						else detec_res = No_Crash;      //找到路径则按新路径走
					}
					if (detec_res == Crash_type2) {      //类型2采取回退策略
						if (0 < robot_path[i].cur_step && robot_path[i].cur_step < robot_path[i].all_steps) {//回退       要保证cur_step不是第一步才能回退
							robot_path[i].cur_step--;
							int command = robot_path[i].commands[robot_path[i].cur_step];  //command0-3
							if (command == 0 || command == 2) command++;   //0->1  2->3   
							else command--;//  1->0   3->2
							printf("move %d %d\n", i, command);
							continue;
						}
						else {            //cur_step是第一步  没法回退了 重新规划路径
							//cout << "--------------------------------------------can_back_replan----" << endl;
							int res = Re_Plan(i, robot[i].dst_type);
							if (res == No_Solution) continue;      //找不到路径则等待
							else detec_res = No_Crash;      //找到路径则按新路径走
						}
					}
					//     No_Crash  正常走
					if (detec_res == No_Crash) {
						int command = robot_path[i].commands[robot_path[i].cur_step];
						printf("move %d %d\n", i, command);
						robot_path[i].cur_step++;
					}
				}
			}
			//移动后 机器人在港口目的点且有货物  放下货物
			//    机器人在港口 或者机器人move后要在港口
			if ((my_map[robot[i].x][robot[i].y] == 'B' && robot[i].have_goods == 1 && robot[i].dst_type == Dst_Berth)\
				|| (robot_path[i].cur_step == robot_path[i].all_steps && robot[i].dst_type == Dst_Berth && robot_path[i].all_steps != 0 && robot[i].have_goods == 1))
			{
				printf("pull %d\n", i);
				all_goods_num++;
				final_money = final_money + robot[i].dst_goods.value;
				clear_path(i);
				berth[robot[i].dst_berth_id].goods_num++;           //港口货物数++
				berth_state[robot[i].dst_berth_id].current_num_goods++;
				berth_state[robot[i].dst_berth_id].current_value_goods += robot[i].dst_goods.value;
				berth_state[robot[i].dst_berth_id].berth_attri.push_back({ id, berth_state[robot[i].dst_berth_id].current_num_goods,berth_state[robot[i].dst_berth_id].current_value_goods });
				robot[i].dst_type = No_Dst;         //放下后变成无目的                   
				robot[i].have_goods = 0;
			}
			// 移动后 机器人在货物处且目前无货物 则取货   
			// 或者机器人move后到货物  
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
		//船的控制
		
		for (int i = 0; i < 5; i++) {
			if (boat[i].status == 1 && boat[i].pos != -1 && (14990 - id) <= berth[boat[i].pos].transport_time) {   //根据运输时间  最后时间不够了 离开
				printf("go %d\n", i);
				continue;
			}
			if (boat[i].status == 0)  continue;            //运输中就先不管
			//int goods_num;  //船上已有货物数量
			//int pos;       // 此时船所在的位置，虚拟点为-1，其余泊位为对应序号
			//int status;   //表示是否到达虚拟点或者泊点, 运输过程中status的值为0，停靠中status的值为1,在泊位外等待的值为2
			//int task_berth = -1;  //初始状态，task_berth表示船对应的泊位*
			//int stay_start;       //到达泊点时的实际帧数*
			//int arrive_flag = -1; //到达泊点标志*
			if (i == 0) {
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth == -1) {  //船为初始化状态
					printf("ship %d 2\n", i);  //去 berth2
					boat[i].task_berth = 2;
					boat[i].task_next_berth = 0;  //船由泊位2去泊位0
					continue;
				}
				if (boat[i].status == 1 && boat[i].pos != -1 && boat[i].arrive_flag == -1) {         //刚到的时候记录起始帧
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
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth != -1) {   //船从某个泊点到了虚拟点 
					printf("ship %d 2\n", i);
					continue;
				}
			}
			if (i == 1) {
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth == -1) {  //船为初始化状态
					printf("ship %d 7\n", i);  //去 berth7
					boat[i].task_berth = 7;
					boat[i].task_next_berth = 8;  //船由泊位7去泊位8
					continue;
				}
				if (boat[i].status == 1 && boat[i].pos != -1 && boat[i].arrive_flag == -1) {         //刚到的时候记录起始帧
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
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth != -1) {   //船从某个泊点到了虚拟点 
					printf("ship %d 7\n", i);
					continue;
				}
			}
			if (i == 2) {
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth == -1) {  //船为初始化状态
					printf("ship %d 4\n", i);  //去 berth6
					boat[i].task_berth = 4;
					boat[i].task_next_berth = 9;  //船由泊位6去泊位3
					continue;
				}
				if (boat[i].status == 1 && boat[i].pos != -1 && boat[i].arrive_flag == -1) {         //刚到的时候记录起始帧
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
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth != -1) {   //船从某个泊点到了虚拟点 
					printf("ship %d 4\n", i);
					continue;
				}
			}
			if (i == 3) {
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth == -1) {  //船为初始化状态
					printf("ship %d 6\n", i);  //去 berth8
					boat[i].task_berth = 6;
					boat[i].task_next_berth = 1;  //船由泊位8去泊位5
					continue;
				}
				if (boat[i].status == 1 && boat[i].pos != -1 && boat[i].arrive_flag == -1) {         //刚到的时候记录起始帧
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
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth != -1) {   //船从某个泊点到了虚拟点 
					printf("ship %d 6\n", i);
					continue;
				}
			}
			if (i == 4) {
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth == -1) {  //船为初始化状态
					printf("ship %d 5\n", i);  //去 berth9
					boat[i].task_berth = 5;
					boat[i].task_next_berth = 3;  //船由泊位9去泊位7
					continue;
				}
				if (boat[i].status == 1 && boat[i].pos != -1 && boat[i].arrive_flag == -1) {         //刚到的时候记录起始帧
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
				if (boat[i].pos == -1 && boat[i].status == 1 && boat[i].task_berth != -1) {   //船从某个泊点到了虚拟点 
					printf("ship %d 5\n", i);
					continue;
				}
			}

			//if (boat[i].status == 1 && boat[i].pos != -1 && (14990 - id) <= berth[boat[i].pos].transport_time) {   //根据运输时间  最后时间不够了 离开
			//	printf("go %d\n", i);
			//	continue;
			//}
			////船在虚拟点 ship到一个没船去且货最多的泊位
			//if (boat[i].status == 1 && boat[i].pos == -1) {
			//	boat[i].goods_num = 0;     //货物置0
			//	int berth_id = boat_search_berth(i);
			//	if (berth_id != Berth_NoGoods) {
			//		printf("ship %d %d\n", i, berth_id);
			//		continue;
			//	}
			//}
			////船满了去虚拟点        go之后不结算物品更新信息 
			//if ((boat[i].pos != -1) && (boat[i].status == 1) && (boat_capacity == boat[i].goods_num)) {
			//	berth[boat[i].pos].now_loading = 0;
			//	printf("go %d\n", i);
			//	continue;
			//}
			////	 船所在的berth没东西了 去其他泊位
			//if ((boat[i].pos != -1) && (berth[boat[i].pos].goods_num == 0)) {
			//	berth[boat[i].pos].now_loading = 0;
			//	int berth_id = boat_search_berth(i);
			//	if (berth_id != Berth_NoGoods) {
			//		printf("ship %d %d\n", i, berth_id);
			//		continue;
			//	}
			//}
			////船在泊位上   更新船和berth的物品数
			//if ((boat[i].pos != -1) && (boat[i].status == 1)) {
			//	if (berth[boat[i].pos].goods_num > berth[boat[i].pos].loading_speed) {       //够不够装一次 
			//		// 装一次会不会溢出
			//		if (boat_capacity < berth[boat[i].pos].loading_speed) {
			//			boat[i].goods_num += berth[boat[i].pos].loading_speed;
			//			berth[boat[i].pos].goods_num -= berth[boat[i].pos].loading_speed;
			//		}
			//		else {        //会溢出时
			//			berth[boat[i].pos].goods_num = berth[boat[i].pos].goods_num - (boat_capacity - boat[i].goods_num);
			//			boat[i].goods_num = boat_capacity;
			//		}
			//	}
			//	else {  // 不够装一次
			//		boat[i].goods_num += berth[boat[i].pos].goods_num;
			//		berth[boat[i].pos].goods_num = 0;
			//	}
			//}
		}
		zhen = id;   //帧号更新为当前实际帧号
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