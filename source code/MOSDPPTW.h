
#include<vector>
using namespace std;

#ifndef VRPTW_H
#define VRPTW_H

const int MAX = 3000;
const int FUNC_NUM = 5;
const int INF = 9999999;
const double EPS = 0.000001;

int data_set;
double stop_criteria = 180;
double max_delay_time = 1800;
double max_capacity = 200;

static struct Customer {
	int id;
	double x;
	double y;
	double b_time;
	double e_time;
	double s_time;
	double p_demand;
	double d_demand;
}customer[MAX];

double a_time[MAX];                         //arrival time
double l_time[MAX];                         //leave time
double w_time[MAX];                         //wait time
double d_time[MAX];                          //delay time
double total_wait[MAX];                     //the total wait time before i
double total_delay[MAX];                     //the total delay time before i
double max_wait[MAX];

double b_demand[MAX];                      // the total demand before customer i including i
double a_demand[MAX];                      // the total demand after customer i including i

double b_distance[MAX];                    //the total distance before customer i including i
double a_distance[MAX];                    //the total distance after customer i including i


struct Route {
	vector<int> customers;
	double travel_dist;
	double travel_time;
	double delay_time;
	double wait_time;
};

int cust_num;
int obj_num = 5;

double peer_time[MAX][MAX];
double peer_distance[MAX][MAX];

int archive_size = 500;
/*double epsilon[FUNC_NUM];  */            //epsilon ±£´æ¼«Öµ
double epsilon = 0.005;

int H = 4;
int weights_size = 0;
int oper_num = 7;
double radius = 0;

struct Weights {
	double lambda[MAX];
	double prob_oper[MAX];
	int invoke_num_oper[MAX];
	double invoke_num;
} weights[MAX];

vector<int> sort_b_time;
int cust_min_time[MAX][MAX];

bool used[MAX][MAX];
struct Chromosome {
	vector<Route> routes;
	double f[FUNC_NUM];       
	double wf;
	double nor_fit[FUNC_NUM];
	bool operator < (const Chromosome &chrome) const{
		return f[0] < chrome.f[0] || (f[0] == chrome.f[0] && f[1] < chrome.f[1]);
	}

	double similarity;
	double crowding;
};

int EP_SIZE = 500;

double IGD_value[MAX];
double IGD_time[MAX];
double frac_time;
vector<Chromosome> ref_p;

vector<Chromosome>total_best;
Chromosome extreme[FUNC_NUM];

vector<Chromosome>pop;
vector<Chromosome> EP;
vector<Chromosome>parent;
vector<Chromosome>children;
vector< vector<int> > F;
vector< vector<int> > S;
vector<Chromosome> chrome_list;//tabu_chrome;
int LIST_SIZE = 50;

int tabu_len;
int ITER = 3000;
int pop_size = 70;

bool EP_flag;

#endif
