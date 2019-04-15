#include "MOSDPPTW.h"
#include<stdio.h>
#include<time.h>
#include<stdlib.h>
#include<math.h>
#include<string.h>
#include<vector>
#include<string>
#include<algorithm>
#include<iostream>
#include<fstream>
using namespace std;

double convert_time(string s_time) {              //second
	char hour[3], minute[3];
	int n_hour, n_minute; double n_time = 0;
	hour[0] = s_time[0]; hour[1] = s_time[1]; hour[2] = '\0';
	minute[0] = s_time[3]; minute[1] = s_time[4]; minute[2] = '\0';
	n_hour = atoi(hour);
	n_minute = atoi(minute);
	n_time = n_hour * 60 + n_minute;
	return n_time * 60 - 28800;
}

void getData(char in_distance[], char in_spec[]) {
	max_capacity = 2.5;
	fstream r_distance(in_distance, ios::in), r_spec(in_spec, ios::in);
	char line[300];
	int id, from_node, to_node; double distance, time;
	r_distance.getline(line, 300);
	while (!r_distance.eof()) {
		r_distance >> id >> from_node >> to_node >> distance >> time;
		from_node %= 10000;
		to_node %= 10000;
		peer_time[from_node][to_node] = time;
		peer_distance[from_node][to_node] = distance;
	}
	r_distance.close();
	int type; double vol, weight, x, y; string b_time, e_time;
	cust_num = 0;
	r_spec.getline(line, 300);
	while (!r_spec.eof()) {
		r_spec >> id;
		id %= 10000;
		r_spec >> type;
		r_spec >> x >> y;
		r_spec >> weight >> vol;
		r_spec >> b_time >> e_time;
		if (type != 4) {
			if (id != 0) ++cust_num;
			customer[id].id = id;
			customer[id].x = x;
			customer[id].y = y;
			customer[id].b_time = convert_time(b_time);
			customer[id].e_time = convert_time(e_time);
			customer[id].s_time = 1800;
			if (id == 0) customer[id].s_time = 0;
			customer[id].p_demand = 0;
			customer[id].d_demand = 0;
			if (type == 2) {
				customer[id].d_demand = weight;
			}
			if (type == 3) {
				customer[id].p_demand = weight;
			}
		}
		
	}
	r_spec.close();

}

void getData(char distance[], char time[], char spec[]) {

	fstream r_distance(distance, ios::in), r_time(time, ios::in), r_spec(spec, ios::in);
	char line[300];
	for (int i = 0;i<4;++i) r_spec.getline(line, 300);
	for (int i = 0;i<2;++i) r_spec >> max_capacity;
	for (int i = 0;i<4;++i) r_spec.getline(line, 300);
	for (int i = 0;i <= cust_num;++i) {
		r_spec >> customer[i].id;
		r_spec >> customer[i].x;
		r_spec >> customer[i].y;
		r_spec >> customer[i].d_demand;
		r_spec >> customer[i].p_demand;
		r_spec >> customer[i].b_time;
		r_spec >> customer[i].e_time;
		r_spec >> customer[i].s_time;
	}
	r_spec.close();
	for (int i = 0;i <= cust_num;++i)
		for (int j = 0;j <= cust_num;++j) {
			r_time >> peer_time[i][j];
			r_distance >> peer_distance[i][j];
		}
	r_time.close();
	r_distance.close();
}

void dfs(int index, int sum, int temp[], int t_array[], int &weights_size,  const int &H) {                                  //calculate lamda by dfs
	int i = 0;
	if (index == obj_num) {
		if (sum == H) {
			for (int j = 0; j < obj_num; ++j) {
			 weights[weights_size].lambda[j] = temp[j] / (H * 1.0);
			}
			++weights_size;
		}
		return;
	}
	if (sum > H) return;
	for (i = 0; i <= H; ++i) {
		temp[index] = t_array[i];
		dfs(index + 1, sum + t_array[i], temp, t_array, weights_size, H);
	}
}

void compute_f(Chromosome &chrome) {
	int i = 0, size = chrome.routes.size();
	for (i = 0; i < obj_num; ++i) {
		chrome.f[i] = 0;
	}
	for (i = 0; i < size; ++i) {
		++chrome.f[0];    //vehicle num
		chrome.f[1] += chrome.routes[i].travel_dist;    //total travel distance
		if (chrome.f[2] < chrome.routes[i].travel_time) chrome.f[2] = chrome.routes[i].travel_time;  //makespan
		chrome.f[3] += chrome.routes[i].wait_time;  //total weight time
		chrome.f[4] += chrome.routes[i].delay_time; // total delay time
	}
}


double compute_wf(double fit[], double r_lamda[], double max_num[], double min_num[]) {
	double sum = 0;
	for (int i = 0; i < FUNC_NUM; ++i) sum += r_lamda[i] * (fit[i] - min_num[i]) / (max_num[i] - min_num[i] + 1);
	return sum;
}

void cal_route_cost(Route &route) {
	int i = 0, size = route.customers.size();
	route.delay_time = 0;
	route.wait_time = 0;
	route.travel_dist = 0;
	route.travel_time = 0;
	double t_time = 0;
	for (i = 1; i < size; ++i) {
		route.travel_dist += peer_distance[route.customers[i - 1]][route.customers[i]];
		t_time += peer_time[route.customers[i - 1]][route.customers[i]];
		if (t_time < customer[route.customers[i]].b_time) {
			route.wait_time += customer[route.customers[i]].b_time - t_time;
			t_time = customer[route.customers[i]].b_time;
		}
		if (t_time > customer[route.customers[i]].e_time) {
			route.delay_time += t_time - customer[route.customers[i]].e_time;
		}
		t_time += customer[route.customers[i]].s_time;
	}
	route.travel_time = t_time;
	
}

bool check_feasible(Route & route) {
	int i = 0, last_cust = 0, cust = 0, size = route.customers.size();
	double t_time = 0;
	for (i = 1; i < size; ++i) {
		cust = route.customers[i];
		t_time += peer_time[last_cust][cust];	
		if (t_time < customer[cust].b_time) {
			t_time = customer[cust].b_time;
		}
		if (cust != 0 && t_time > customer[cust].e_time + max_delay_time) {
	//		printf("time error!\n");
			return false;
		}
		else if (cust == 0 && t_time > customer[cust].e_time) {
			return false;
		}
	    t_time += customer[cust].s_time;
		last_cust = cust;
	}
	double delivery_weight = 0; 
	for (i = 1; i < size; ++i) {
		cust = route.customers[i];
		delivery_weight += customer[cust].d_demand;
	}
	if (delivery_weight > max_capacity) return false;
	double load = delivery_weight;
	for (i = 1; i < size; ++i) {
		cust = route.customers[i];
		load -= customer[cust].d_demand;
		load += customer[cust].p_demand;
		if (load > max_capacity) return false;
	}
	cal_route_cost(route);
	return true;
}

bool check_chromosome(Chromosome &chrome) {
	bool used[MAX];
	vector<Route>::iterator r_iter;
	for (int j = 0;j <= cust_num;++j) used[j] = false;
	for (r_iter = chrome.routes.begin(); r_iter != chrome.routes.end();++r_iter) {
		double t_time = 0;
		double w_time = 0;
		double d_capacity = 0;
		if (r_iter->customers[0] != 0 || r_iter->customers[r_iter->customers.size() - 1] != 0) {
			printf("depot error!\n");
			return 0;
		}
		vector<int>::iterator c_iter;
		for (c_iter = r_iter->customers.begin(); c_iter != r_iter->customers.end() - 1; ++c_iter) {
			d_capacity += customer[*c_iter].d_demand;
			if (d_capacity > max_capacity) {
				printf("delivery capacity false! \n");
				return false;
			}
		}
		for (c_iter = r_iter->customers.begin() + 1; c_iter != r_iter->customers.end() - 1; ++c_iter) {
			d_capacity = d_capacity - customer[*c_iter].d_demand + customer[*c_iter].p_demand;
			if (d_capacity > max_capacity) {
				printf("pickup capacity false! %lf\n", d_capacity);
				exit(1);
				return false;
			}
		}
		for (c_iter = r_iter->customers.begin(); c_iter != r_iter->customers.end() - 1;++c_iter) {
			t_time += peer_time[*c_iter][*(c_iter + 1)];
			if (*(c_iter + 1) != 0) {
				if (t_time>customer[*(c_iter + 1)].e_time + max_delay_time) {
					printf("time false!!\n");
					return false;
				}
			}
			else {
				if (t_time>customer[*(c_iter + 1)].e_time) {
					printf("end time false!!\n");
					return false;
				}
			}
			w_time = t_time> customer[*(c_iter + 1)].b_time ? 0 : customer[*(c_iter + 1)].b_time - t_time;
			t_time += w_time + customer[*(c_iter + 1)].s_time;
			if (!used[*c_iter]) used[*c_iter] = true;
			else if(*c_iter!=0){
				printf("customer duplicated!\n");
				return false;
			}
		}
	}
	int count = 0;
	for (int j = 1;j <= cust_num;++j) if (used[j]) ++count;
	if (count != cust_num) {
		printf("less customer! \n");
		return false;
	}
	return true;
}

void get_RP(int num, int RP[]) {                                                                  //get a permutation of the std::vector
	for (int i = 0; i < num; ++i) RP[i] = i;
	for (int i = 0; i < num; ++i) {
		int r0 = i + rand() % (num - i);
		swap(RP[i], RP[r0]);
	}
}

/////////////////////////////arichive///////////////////////////////////////

bool is_equal (Chromosome &chrome1, Chromosome &chrome2) {
	int i = 0;
	for (i = 0; i < obj_num; ++i) {
		if (fabs(chrome1.f[i] - chrome2.f[i]) > EPS) return false;
	}
	return true;
}

bool is_better(Chromosome &chrome1, Chromosome &chrome2) {
	bool better = false; int i = 0;
	for (i = 0; i < obj_num; ++i) {
		if (chrome1.f[i] - chrome2.f[i] > EPS) return false;
		else if (chrome2.f[i] - chrome1.f[i] > EPS ) better = true;
	}
	return better;
}

bool update_EP(vector<Chromosome> &EP, Chromosome &chrome) {
	if (EP.size() == 0) {
		EP.push_back(chrome);
		return true;
	}
	int i = 0, j = 0, size = EP.size();
	for (i = 0; i < size; ++i) {
		if (is_better(EP[i], chrome) || is_equal (EP[i], chrome)) return false;
	}
	for (i = 0; i < EP.size();) {
		if (is_better(chrome, EP[i])) {
			EP.erase(EP.begin() + i);
		}
		else ++i;
	}
	EP.push_back(chrome);
}

//////////////////////////////////////////////////////////////////////

///////////////////neighboorhood operators//////////////////////////

void insert_empty_route(Chromosome &chrome, Route &route) {
	route.customers.push_back(0);
	route.customers.push_back(0);
	cal_route_cost(route);
	chrome.routes.push_back(route);
	++chrome.f[0];
}

void remove_empty_route(Chromosome &chrome) {
	int i = 0, size = chrome.routes.size();
	for (i = 0; i < size;) {
		if (chrome.routes[i].customers.size() == 2) {
			chrome.routes.erase(chrome.routes.begin() + i);
			--size;
		}
		else ++i;
	}
}

void find_max_min(vector<Chromosome>& EP, double max_num[], double min_num[]) {
	int i = 0, j = 0, size = EP.size();
	for (i = 0; i < obj_num; ++i) {
		max_num[i] = -INF;
		min_num[i] = INF;
	}
	for (i = 0; i < size; ++i) {
		for (j = 0; j < obj_num; ++j) {
			if (EP[i].f[j] > max_num[j]) max_num[j] = EP[i].f[j];
			if (EP[i].f[j] < min_num[j]) min_num[j] = EP[i].f[j];
		}
	}
}

void cal_part_fitness(Chromosome &chrome, double fitness[], vector<int> route_index) {
	int i = 0, size = route_index.size();
	for (i = 0; i < obj_num; ++i) fitness[i] = 0;
	for (i = 0; i < size; ++i) {
		int j = route_index[i];
		++fitness[0];
		fitness[1] += chrome.routes[j].travel_dist;
		if (fitness[2] < chrome.routes[j].travel_time) fitness[2] = chrome.routes[j].travel_time;
		fitness[3] += chrome.routes[j].wait_time;
		fitness[4] += chrome.routes[j].delay_time;
	}

}

void insert_customer(Chromosome &chrome, int cust, double lambda[], double max_num[], double min_num[]) {                    //insert a customer to the best place in a route plan
	Route e_route;
	insert_empty_route(chrome, e_route);
	int i = 0, j = 0, size = chrome.routes.size();
	int insert_route = -1, insert_pos = -1;
	double fitness[MAX], oper_fit[MAX];
	double best_fit = INF;
	vector<int> route_index;
	for (i = 0; i < size; ++i) {
		route_index.clear();
		for (j = 0; j < size; ++j) {
			if (j != i) route_index.push_back(j);
		}
		cal_part_fitness(chrome, fitness, route_index);
		int cust_size = chrome.routes[i].customers.size();
		for (j = 1; j < cust_size ; ++j){
			Route temp_route = chrome.routes[i];
			temp_route.customers.insert(temp_route.customers.begin() + j, cust);
			if (check_feasible(temp_route)) {
				oper_fit[0] = fitness[0] + 1;
				oper_fit[1] = fitness[1] + temp_route.travel_dist;
				oper_fit[2] = temp_route.travel_time > fitness[2] ? temp_route.travel_time : fitness[2];
				oper_fit[3] = fitness[3] + temp_route.wait_time;
				oper_fit[4] = fitness[4] + temp_route.delay_time;
				double fit = compute_wf(oper_fit, lambda, max_num, min_num);
				if (best_fit - fit > EPS) {
					best_fit = fit;
					insert_route = i;
					insert_pos = j;
				}
			}
		}
		
	}
	chrome.routes[insert_route].customers.insert(chrome.routes[insert_route].customers.begin() + insert_pos, cust);
	cal_route_cost(chrome.routes[insert_route]);
	remove_empty_route(chrome);
	compute_f(chrome);
	//if (insert_route == -1) {
	//	Route temp_route;
	//	temp_route.customers.push_back(0);
	//	temp_route.customers.push_back(cust);
	//	temp_route.customers.push_back(0);
	//	cal_route_cost(temp_route);
	//	chrome.routes.push_back(temp_route);
	//	compute_f(chrome);
	//}
	//else {
	//	chrome.routes[insert_route].customers.insert(chrome.routes[insert_route].customers.begin() + insert_pos, cust);
	//	cal_route_cost(chrome.routes[insert_route]);
	//	remove_empty_route(chrome);
	//	compute_f(chrome);
	//}

}


void one_allocate(Chromosome &chrome, double lambda[], double max_num[], double min_num[]) {
	int select_route_index = rand() % chrome.routes.size();
	int select_cust_index = rand() % (chrome.routes[select_route_index].customers.size() - 2) + 1;
	int cust = chrome.routes[select_route_index].customers[select_cust_index];
	chrome.routes[select_route_index].customers.erase(chrome.routes[select_route_index].customers.begin() + select_cust_index);
	if (chrome.routes[select_route_index].customers.size() == 2) chrome.routes.erase(chrome.routes.begin() + select_route_index);
	else cal_route_cost(chrome.routes[select_route_index]);
	compute_f(chrome);
	insert_customer(chrome, cust, lambda, max_num, min_num);
}

void several_allocate(Chromosome &chrome, double lambda[], double max_num[], double min_num[]) {
	int cust_iter = cust_num * 0.1, i = 0, select_route_index = 0, select_cust_index = 0, cust = 0;
	vector<int> custs;
	for (i = 0; i < cust_iter; ++i) {
		select_route_index = rand() % chrome.routes.size();
		select_cust_index = rand() % (chrome.routes[select_route_index].customers.size() - 2) + 1;
		cust = chrome.routes[select_route_index].customers[select_cust_index];
		chrome.routes[select_route_index].customers.erase(chrome.routes[select_route_index].customers.begin() + select_cust_index);
		if (!check_feasible(chrome.routes[select_route_index])){
			chrome.routes[select_route_index].customers.insert(chrome.routes[select_route_index].customers.begin() + select_cust_index, cust);
		}
		else custs.push_back(cust);
		if (chrome.routes[select_route_index].customers.size() == 2) chrome.routes.erase(chrome.routes.begin() + select_route_index);
		else cal_route_cost(chrome.routes[select_route_index]);
	}
	compute_f(chrome);
	cust_iter = custs.size();
	for (i = 0; i < cust_iter; ++i) {
		insert_customer(chrome, custs[i], lambda, max_num, min_num);
	}
}

void two_opt_star(Chromosome &chrome, double lambda[], double max_num[], double min_num[]) {
	int i = 0, j = 0, k = 0, size = chrome.routes.size();
	int select_route_index = rand() % size;
	int select_cust_size = chrome.routes[select_route_index].customers.size();
	int select_cust_index = rand() % (select_cust_size - 2) + 1;
	double oper_fit[MAX];
	double best_fit = compute_wf(chrome.f, lambda, max_num, min_num);
	vector<int> route_index;
	int insert_route = -1, insert_pos = -1;
	double fitness[MAX];
	Route temp_route1, temp_route2, e_route;
	insert_empty_route(chrome, e_route);
	++size;
	for (i = 0; i < size; ++i) {
		if (i != select_route_index) {
			route_index.clear();
			for (j = 0; j < size; ++j) {
				if (j != select_route_index && j != i) route_index.push_back(j);
			}
			cal_part_fitness(chrome, fitness, route_index);
			int cust_size = chrome.routes[i].customers.size();
			for (j = 0; j < cust_size - 1; ++j) {
				temp_route1.customers.clear();
				temp_route2.customers.clear();
				for (k = 0; k <= j; ++k) temp_route1.customers.push_back(chrome.routes[i].customers[k]);
				for (k = select_cust_index + 1; k < select_cust_size; ++k) temp_route1.customers.push_back(chrome.routes[select_route_index].customers[k]);
				for (k = 0; k <= select_cust_index; ++k) temp_route2.customers.push_back(chrome.routes[select_route_index].customers[k]);
				for (k = j + 1; k < cust_size; ++k) temp_route2.customers.push_back(chrome.routes[i].customers[k]);
				if (check_feasible(temp_route1) && check_feasible(temp_route2)) {
					oper_fit[0] = fitness[0] + 2;
					oper_fit[1] = fitness[1] + temp_route1.travel_dist + temp_route2.travel_dist;
					double max_fit = fitness[2];
					if (max_fit < temp_route2.travel_time) max_fit = temp_route2.travel_time;
					if (max_fit < temp_route1.travel_time) max_fit = temp_route1.travel_time;
					oper_fit[2] = max_fit;
					oper_fit[3] = fitness[3] + temp_route1.wait_time + temp_route2.wait_time;
					oper_fit[4] = fitness[4] + temp_route1.delay_time + temp_route2.delay_time;
					double fit = compute_wf(oper_fit, lambda, max_num, min_num);
					if (best_fit - fit > EPS) {
						best_fit = fit;
						insert_route = i;
						insert_pos = j;
					}
				}
			}
		}
	}
	if (insert_route != -1) {
		temp_route1.customers.clear();
		temp_route2.customers.clear();
		int cust_size = chrome.routes[insert_route].customers.size();
		for (k = 0; k <= insert_pos; ++k) temp_route1.customers.push_back(chrome.routes[insert_route].customers[k]);
		for (k = select_cust_index + 1; k < select_cust_size; ++k) temp_route1.customers.push_back(chrome.routes[select_route_index].customers[k]);
		for (k = 0; k <= select_cust_index; ++k) temp_route2.customers.push_back(chrome.routes[select_route_index].customers[k]);
		for (k = insert_pos + 1; k < cust_size; ++k) temp_route2.customers.push_back(chrome.routes[insert_route].customers[k]);
		cal_route_cost(temp_route1);
		cal_route_cost(temp_route2);
		if (select_route_index > insert_route) swap(select_route_index, insert_route);
		chrome.routes.erase(chrome.routes.begin() + insert_route);
		chrome.routes.erase(chrome.routes.begin() + select_route_index);
		chrome.routes.push_back(temp_route1);
		chrome.routes.push_back(temp_route2);
	}
	remove_empty_route(chrome);
	compute_f(chrome);
}

void split(Chromosome &chrome, double lambda[], double max_num[], double min_num[]) {
	int i = 0, j = 0, size = chrome.routes.size();
	int select_route_index = rand() % size, cust_size = chrome.routes[select_route_index].customers.size();
	int cust_pos = -1;
	double fitness[MAX], oper_fit[MAX];
	chrome.wf = compute_wf(chrome.f, lambda, max_num, min_num);
	double best_fit = chrome.wf;
	vector<int> index;
	for (i = 0; i < size; ++i) {
		if (i != select_route_index) index.push_back(i);
	}
	cal_part_fitness(chrome, fitness, index);
	for (i = 1; i < cust_size - 1; ++i) {
		Route temp_route1, temp_route2;
		for (j = 0; j <= i; ++j) temp_route1.customers.push_back(chrome.routes[select_route_index].customers[j]);
		temp_route1.customers.push_back(0);
		temp_route2.customers.push_back(0);
		for (j = i + 1; j < cust_size; ++j) temp_route2.customers.push_back(chrome.routes[select_route_index].customers[j]);
		if (check_feasible(temp_route1) && check_feasible(temp_route2)) {
			oper_fit[0] = fitness[0] + 2;
			oper_fit[1] = fitness[1] + temp_route1.travel_dist + temp_route2.travel_dist;
			double max_fit = fitness[2];
			if (max_fit < temp_route2.travel_time) max_fit = temp_route2.travel_time;
			if (max_fit < temp_route1.travel_time) max_fit = temp_route1.travel_time;
			oper_fit[2] = max_fit;
			oper_fit[3] = fitness[3] + temp_route1.wait_time + temp_route2.wait_time;
			oper_fit[4] = fitness[4] + temp_route1.delay_time + temp_route2.delay_time;
			double fit = compute_wf(oper_fit, lambda, max_num, min_num);
			if (best_fit - fit > EPS) {
				best_fit = fit;
				cust_pos = i;
			}
		}

	}
	if (cust_pos != -1) {
		Route temp_route1, temp_route2;
		for (j = 0; j <= cust_pos; ++j) temp_route1.customers.push_back(chrome.routes[select_route_index].customers[j]);
		temp_route1.customers.push_back(0);
		temp_route2.customers.push_back(0);
		for (j = cust_pos + 1; j < cust_size; ++j) temp_route2.customers.push_back(chrome.routes[select_route_index].customers[j]);
		cal_route_cost(temp_route1);
		cal_route_cost(temp_route2);
		chrome.routes.erase(chrome.routes.begin() + select_route_index);
		chrome.routes.push_back(temp_route1);
		chrome.routes.push_back(temp_route2);
		compute_f(chrome);
	}
} 

void merge(Chromosome &chrome, double lambda[], double max_num[], double min_num[]) {
	int i = 0, j = 0, size = chrome.routes.size();
	int select_route_index = -1, min_cust = INF; 
	for (i = 0; i < size; ++i) {
		if (chrome.routes[i].customers.size() < min_cust) {
			min_cust = chrome.routes[i].customers.size();
			select_route_index = i;
		}
	}
	vector<int> custs;
	int cust_size = chrome.routes[select_route_index].customers.size();
	for (j = 1; j < cust_size - 1; ++j) {
		custs.push_back(chrome.routes[select_route_index].customers[j]);
	}
	chrome.routes.erase(chrome.routes.begin() + select_route_index);
	compute_f(chrome);
	for (i = 0; i < custs.size(); ++i) {
		insert_customer(chrome, custs[i], lambda, max_num, min_num);
	}
}

void reverse(Chromosome &chrome, double lambda[], double max_num[], double min_num[]) {
	int i = 0, j = 0, size = chrome.routes.size();
	double fitness[MAX], oper_fit[MAX];
	chrome.wf = compute_wf(chrome.f, lambda, max_num, min_num);
	for (i = 0; i < size; ++i) {
		Route temp_route = chrome.routes[i];
		vector<int> route_index;
		for (j = 0; j < size; ++j) {
			if (j != i) route_index.push_back(j);
		}
		cal_part_fitness(chrome, fitness, route_index);
		int cust_size = chrome.routes[i].customers.size();
		if (cust_size >= 4) {
			for (j = 1; j < cust_size - 2; ++j) {
				swap(temp_route.customers[j], temp_route.customers[j + 1]);
				if (check_feasible(temp_route)) {
					oper_fit[0] = fitness[0] + 1;
					oper_fit[1] = fitness[1] + temp_route.travel_dist;
					oper_fit[2] = temp_route.travel_time > fitness[2] ? temp_route.travel_time : fitness[2];
					oper_fit[3] = fitness[3] + temp_route.wait_time;
					oper_fit[4] = fitness[4] + temp_route.delay_time;
					double fit = compute_wf(oper_fit, lambda, max_num, min_num);
					if (chrome.wf - fit > EPS) {
						chrome.routes.erase(chrome.routes.begin() + i);
						chrome.routes.push_back(temp_route);
						compute_f(chrome);
						return;
					}
				}
				swap(temp_route.customers[j], temp_route.customers[j + 1]);
			}
		}
	}
}

void two_opt_route(Chromosome &chrome, int index, double lambda[], double max_num[], double min_num[]) {
	int i = 0, j = 0, route_size = chrome.routes.size(), cust_size = chrome.routes[index].customers.size();
	int len = 1; vector<int> route_index;
	chrome.wf = compute_wf(chrome.f, lambda, max_num, min_num);
	double min_cost = chrome.wf;
	double fitness[MAX], oper_fit[MAX];
	for (i = 0; i < route_size; ++i) {
		if (i != index) {
			route_index.push_back(i);
		}
	}
	cal_part_fitness(chrome, fitness, route_index);
	Route best_route = chrome.routes[index];
	while (len < cust_size - 1) {
		Route temp_route;
		for (i = 0; i <= len; ++i) {
			temp_route.customers.push_back(chrome.routes[index].customers[i]);
		}
		int reverse_len = 1;
		while (len + 1 + reverse_len < cust_size - 1) {
			Route reverse_route = temp_route;
			for (i = len + 1 + reverse_len; i >= len + 1; --i) {
				reverse_route.customers.push_back(chrome.routes[index].customers[i]);
			}
			for (i = len + reverse_len + 2; i < cust_size; ++i) {
				reverse_route.customers.push_back(chrome.routes[index].customers[i]);
			}
			if (check_feasible(reverse_route)) {
				oper_fit[0] = fitness[0] + 1;
				oper_fit[1] = fitness[1] + reverse_route.travel_dist;
				oper_fit[2] = reverse_route.travel_time > fitness[2] ? reverse_route.travel_time : fitness[2];
				oper_fit[3] = fitness[3] + reverse_route.wait_time;
				oper_fit[4] = fitness[4] + reverse_route.delay_time;
				double fit = compute_wf(oper_fit, lambda, max_num, min_num);
				if (min_cost - fit > EPS) {
					min_cost = fit;
					best_route = reverse_route;
				}
			}
			++reverse_len;
		}
		++len;
	}
	chrome.routes[index] = best_route;
}

void two_opt(Chromosome &chrome, double lambda[], double max_num[], double min_num[]) {
	int route_size = chrome.routes.size();
	int route_index = rand() % route_size;
	two_opt_route(chrome, route_index, lambda,  max_num, min_num);
	compute_f(chrome);
	//	check_feasible_chrome(chrome);
}

void remove_customer(Chromosome &chrome, double lambda[], double max_num[], double min_num[]) {
	int route_size = chrome.routes.size();
	int route_index = rand() % route_size;
	int cust_size = chrome.routes[route_index].customers.size();
	int cust_index = rand() % (cust_size - 2) + 1;
	int cust = chrome.routes[route_index].customers[cust_index];
	chrome.routes[route_index].customers.erase(chrome.routes[route_index].customers.begin() + cust_index);
	if (check_feasible(chrome.routes[route_index])) {
		Route temp_route;
		temp_route.customers.push_back(0);
		temp_route.customers.push_back(cust);
		temp_route.customers.push_back(0);
		cal_route_cost(temp_route);
		chrome.routes.push_back(temp_route);
	}
	else {
		chrome.routes[route_index].customers.insert(chrome.routes[route_index].customers.begin() + cust_index, cust);
		cal_route_cost(chrome.routes[route_index]);
	}
	compute_f(chrome);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

double cal_Eculid(double lambda1[], double lambda2[]) {
	int i = 0; double sum = 0;
	for (i = 0; i < obj_num; ++i) {
		sum += (lambda1[i] - lambda2[i]) * (lambda1[i] - lambda2[i]);
	}
	return sqrt(sum);
}

int find_lambda_index(double lambda[]) {
	int i = 0, index = 0; double sum = 0, min_cost = INF;
	for (i = 0; i < weights_size; ++i) {
		sum = cal_Eculid(lambda, weights[i].lambda);
		if (min_cost - sum > EPS) {
			min_cost = sum;
			index = i;
		}
	}
	return index;
}

int wheel_routte(double num[], int bound_num) {
	double prob[MAX];
	int i = 0;
	prob[0] = num[0];
	for (i = 1; i < bound_num; ++i) {
		prob[i] = prob[i - 1] + num[i];
	}
	for (i = 0; i < bound_num; ++i) {
		prob[i] /= prob[bound_num - 1];
	}
	double p = rand() % 1000 / 1000.0;
	if (p <= prob[0]) return 0;
	for (i = 0; i < bound_num - 1; ++i) {
		if (p > prob[i] && p <= prob[i + 1]) return i + 1;
	}
}

bool local_search(Chromosome &chrome, double lambda[], double max_num[], double min_num[]) {
	int i = 0, j = 0, ITER = 50; double beta = 0.1;
	int oper_index = 0;
	bool flag = false;
	void(*f[20])(Chromosome & chrome, double lambda[], double max_num[], double min_num[]);
	f[0] = one_allocate;
	f[1] = several_allocate;
	f[2] = two_opt_star;
	f[3] = split;
	f[4] = merge;
	f[5] = reverse;
	f[6] = two_opt;
//	f[7] = remove_customer;
	int lambda_index = find_lambda_index(lambda);
	chrome.wf = compute_wf(chrome.f, lambda, max_num, min_num);
	double count[MAX], invoke[MAX];
	for (i = 0; i < oper_num; ++i) {
		count[i] = 0;
		invoke[i] = 0;
	}
	Chromosome best_chrome = chrome;
	for (i = 0; i < 50; ++i) {
//		double p = rand() % 1000 / 1000.0;
//		if (p > weights[lambda_index].prob_oper[i]) continue;
   //     oper_index = rand() % oper_num;
        oper_index =  wheel_routte(weights[lambda_index].prob_oper, oper_num);
		++weights[lambda_index].invoke_num_oper[oper_index];
		f[oper_index](chrome, lambda, max_num, min_num);
	//	check_chromosome(chrome);
		chrome.wf = compute_wf(chrome.f, lambda, max_num, min_num);
		++invoke[oper_index];
		if (best_chrome.wf - chrome.wf > EPS) {
			best_chrome = chrome;
			flag = true;
			++count[oper_index];
			//		if(oper_index == 3)printf("%d\n", oper_index);
		}
		else {
			chrome = best_chrome;
	//		for (j = 0; j < weights_size;++j) {
	//			if (j != lambda_index) ++weights[j].prob_oper[oper_index];
	//		}
	//		--weights[lambda_index].prob_oper[oper_index];
	//		if (weights[lambda_index].prob_oper[oper_index] == 0) weights[lambda_index].prob_oper[oper_index] = 1;
		//	weights[lambda_index].prob_oper[oper_index] *= 0.99;
		}
	}
	//double sum = 0;
	//for (i = 0; i < oper_num; ++i) sum += count[i];
	//if(sum != 0) for (i = 0; i < oper_num; ++i) count[i] /= sum;
	//for (i = 0; i < oper_num; ++i) weights[lambda_index].prob_oper[i] = (1 - beta) *  weights[lambda_index].prob_oper[i] + beta * count[i];
	for (i = 0; i < oper_num; ++i) {
		if (invoke[i] != 0) count[i] /= invoke[i];
		else count[i] = 0;
	}
	for (i = 0; i < oper_num; ++i) weights[lambda_index].prob_oper[i] = (1 - beta) *  weights[lambda_index].prob_oper[i] + beta * count[i];
//	for (i = 0; i < oper_num; ++i) weights[lambda_index].prob_oper[i] = max(count[i], 0.01);
//	for (i = 0; i < oper_num; ++i) printf("%lf ", count[i]);
//	printf("\n");
	update_EP(EP, chrome);
	return flag;
}

int cust_ind = 1;
bool cmp_cust_time(const int &num1, const int &num2) {
	return 	 peer_distance[cust_ind][num1] < peer_distance[cust_ind][num2] ||
		peer_distance[cust_ind][num1] == peer_distance[cust_ind][num2] && customer[num1].b_time < customer[num2].b_time ||
		peer_distance[cust_ind][num1] == peer_distance[cust_ind][num2] && customer[num1].b_time == customer[num2].b_time && peer_time[cust_ind][num1] < peer_time[cust_ind][num2];

}

void cal_min_cust_time() {
	for (cust_ind = 0; cust_ind <= cust_num; ++cust_ind) {
		int i = 0;
		for (i = 0; i <= cust_num;++i) {
			cust_min_time[cust_ind][i] = i;
		}
		swap(cust_min_time[cust_ind][cust_ind], cust_min_time[cust_ind][cust_num]);
		sort(cust_min_time[cust_ind], cust_min_time[cust_ind] + cust_num, cmp_cust_time);
	}
}

int select_cust(vector<int> &index, int cust) {
	int i = 0, j = 0, size = index.size(), c_index = 0;
	vector<int>::iterator iter;
	for (i = 0; i < cust_num;++i) {
		iter = find(index.begin(), index.end(), cust_min_time[cust][i]);
		if (iter != index.end()) {
			return iter - index.begin();
		}

	}
	return -1;
}

void init_chromosome(Chromosome &chrome) {
	int i = 0, j = 0; 
	vector<int> left_cust;
	cal_min_cust_time();
	for (int i = 1; i <= cust_num; ++i) left_cust.push_back(i);
	Route route;
	while (left_cust.size() != 0) {
		route.customers.clear();
		//int cust_index = select_cust(left_cust, 0);
		int cust_index = rand() % left_cust.size();
		int cust = left_cust[cust_index];
		route.customers.push_back(0);
		route.customers.push_back(cust);
		double t_time = peer_time[0][cust]; double d_capacity = 0, m_capacity = max_capacity, now_capacity = m_capacity;
		if (t_time < customer[cust].b_time) t_time = customer[cust].b_time;
		t_time += customer[cust].s_time;
		d_capacity += customer[cust].d_demand;
		now_capacity = now_capacity - customer[cust].d_demand + customer[cust].p_demand;
		if (now_capacity > m_capacity) {
			m_capacity -= (now_capacity - m_capacity);
			now_capacity = max_capacity;
		}
		int last_cust = cust;
		left_cust.erase(left_cust.begin() + cust_index);
		while (1) {
			//cust_index = select_cust(left_cust, last_cust);
			if (left_cust.size() == 0) break;
			cust_index = rand() % left_cust.size();
		//	if (cust_index == -1) break;
			cust = left_cust[cust_index];
			double t_d_capacity = d_capacity + customer[cust].d_demand;
			double t_now_capacity = now_capacity - customer[cust].d_demand + customer[cust].p_demand;
			double t_now_max_capacity = m_capacity;
			if (t_now_capacity > t_now_max_capacity) {
				t_now_max_capacity -= (t_now_capacity - t_now_max_capacity);
				t_now_capacity = max_capacity;
			}
			double n_time = t_time + peer_time[last_cust][cust];
			if (n_time < customer[cust].b_time) n_time = customer[cust].b_time;
			if (t_d_capacity > t_now_max_capacity     //capacity violation
				|| n_time > customer[cust].e_time + max_delay_time || n_time + customer[cust].s_time + peer_time[cust][0] > customer[0].e_time    //time violation
				) {
				route.customers.push_back(0);
				cal_route_cost(route);
				chrome.routes.push_back(route);
				break;
			}
			else {
				route.customers.push_back(cust);
				d_capacity = t_d_capacity;
				now_capacity = t_now_capacity;
				m_capacity = t_now_max_capacity;
				t_time = n_time + customer[cust].s_time;
				left_cust.erase(left_cust.begin() + cust_index);
				last_cust = cust;
			}
		}
	}
	route.customers.push_back(0);
	cal_route_cost(route);
	chrome.routes.push_back(route);
	compute_f(chrome);
	check_chromosome(chrome);
	update_EP(EP, chrome);
}

void init_pop(vector<Chromosome>&pop) {
	int i = 0;
	for (i = 0; i < pop_size; ++i) {
		Chromosome chrome;
		init_chromosome(chrome);
		pop.push_back(chrome);
		update_EP(EP, chrome);
	}
}

void output(vector<Chromosome> &EP) {
	sort(EP.begin(), EP.end());
	int i = 0, j = 0, size = EP.size(); 
	for (i = 0; i < size; ++i) {
		for (j = 0; j < obj_num; ++j) {
			printf("%lf ", EP[i].f[j]);
		}
		printf("\n");
	}
}

int find_best_solution(vector<Chromosome> &pop, double lambda[], double max_num[], double min_num[]) {
	double min_fit = INF; int index = -1, size = pop.size(), i = 0;
	for (i = 0; i < size; ++i) {
		double fit = compute_wf(pop[i].f, lambda, max_num, min_num);
		if (fit < min_fit) {
			min_fit = fit;
			index = i;
		}
	}
	return index;
}

void generate_rand_lambda(double lambda[]) {
	int RP[MAX], j = 0;
	get_RP(obj_num, RP);
	lambda[RP[0]] = rand() % 100;
	int sum = 100 - lambda[RP[0]];
	for (j = 1; j < obj_num - 1; ++j) {
		if (sum > 0) lambda[RP[j]] = rand() % sum;
		else lambda[RP[j]] = 0;
		sum -= lambda[RP[j]];
	}
	lambda[RP[obj_num - 1]] = sum;
	for (j = 0; j < obj_num; ++j) lambda[j] /= 100;
}

void generate_rand_lambda(double lambda[], double seed, int lambda_index) {
	double p = rand() % 1000 / 1000.0;
	int i = 0;
	if (p < 0.1) {
		int RP[MAX];
		get_RP(obj_num, RP);
		double upper[MAX], lower[MAX];
		for (i = 0; i < obj_num; ++i) {
			upper[i] = weights[lambda_index].lambda[i] + seed;
			if (upper[i] > 1) upper[i] = 1;
			lower[i] = weights[lambda_index].lambda[i] - seed;
			if (lower[i] < 0) lower[i] = 0;
		}
		double sum = 1;
		for (i = 0; i < obj_num; ++i) sum -= lower[i];
		lambda[RP[0]] = (upper[RP[0]] - lower[RP[0]]) * (rand() % 1000 / 1000.0);
		sum -= lambda[RP[0]];
		if (sum < 0) {
			lambda[RP[0]] = sum + lambda[RP[0]];
			sum = 0;
		}
		for (i = 1; i < obj_num; ++i) {
			lambda[RP[i]] = (upper[RP[i]] - lower[RP[i]]) * (rand() % 1000 / 1000.0);
			sum -= lambda[RP[i]];
			if (sum < 0) {
				lambda[RP[i]] = sum + lambda[RP[i]];
				sum = 0;
			}
		}
		if (sum > 0) {
			for (i = 0; i < obj_num; ++i) {
				if (sum > 0) {
					if (upper[RP[i]] - lower[RP[i]] - lambda[RP[i]] >= sum) {
						lambda[RP[i]] += sum;
						break;
					}
					else {
						sum -= (upper[RP[i]] - lower[RP[i]] - lambda[RP[i]]);
						lambda[RP[i]] = upper[RP[i]] - lower[RP[i]];
					}
				}
				else break;
			}
		}
		for (i = 0; i < obj_num; ++i) lambda[i] += lower[i];
	}
	else {
		for (i = 0; i < obj_num; ++i) lambda[i] = weights[lambda_index].lambda[i];
	}
}

void init_weight() {
	int *t_array, temp[MAX], i = 0, j = 0;
	t_array = new int[H + 1];
	weights_size = 0;
	for (i = 0; i <= H; ++i) t_array[i] = i;
	dfs(0, 0, temp, t_array, weights_size, H);
	delete[]t_array;
	for (i = 0; i < weights_size;++i) {
		for (j = 0; j < oper_num; ++j) {
			weights[i].invoke_num_oper[j] = 0;
			weights[i].prob_oper[j] = 1;
			weights[i].invoke_num = 1;
		}
	}
	radius = 1.0/ (H * 2);
}

double cal_dist(double lamda_1[], double lamda_2[]) {                     //Eculid distance
	double dist = 0;
	for (int i = 0; i < obj_num; ++i) dist += (lamda_1[i] - lamda_2[i])*(lamda_1[i] - lamda_2[i]);
	return sqrt(dist);
}

double IGD(std::vector<Chromosome> & EP, std::vector<Chromosome> &ref) {
	double max_fit[MAX], min_fit[MAX]; int i = 0;
	std::vector<Chromosome>::iterator ref_iter, iter;
	for (i = 0; i < obj_num; ++i) {
		max_fit[i] = -INF;
		min_fit[i] = INF;
	}
	for (ref_iter = ref.begin(); ref_iter != ref.end(); ++ref_iter) {
		for (i = 0; i < obj_num; ++i) {
			if (max_fit[i] < ref_iter->f[i]) max_fit[i] = ref_iter->f[i];
			if (min_fit[i] > ref_iter->f[i]) min_fit[i] = ref_iter->f[i];
		}
	}
	for (iter = EP.begin(); iter != EP.end(); ++iter) {
		for (i = 0; i < obj_num; ++i) {
			if (max_fit[i] < iter->f[i]) max_fit[i] = iter->f[i];
			if (min_fit[i] > iter->f[i]) min_fit[i] = iter->f[i];
		}
	}
	for (ref_iter = ref.begin(); ref_iter != ref.end(); ++ref_iter) {
		for (i = 0; i < obj_num; ++i) {
			ref_iter->nor_fit[i] = (ref_iter->f[i] - min_fit[i]) / (max_fit[i] - min_fit[i]);
		}
	}
	for (iter = EP.begin(); iter != EP.end(); ++iter) {
		for (int i = 0; i < obj_num; ++i) {
			iter->nor_fit[i] = (iter->f[i] - min_fit[i]) / (max_fit[i] - min_fit[i]);
		}
	}

	int size = ref.size(); double IGD = 0, min_value = INF, dist = 0;
	for (ref_iter = ref.begin(); ref_iter != ref.end(); ++ref_iter) {
		min_value = INF;
		for (iter = EP.begin(); iter != EP.end(); ++iter) {
			dist = cal_dist(ref_iter->nor_fit, iter->nor_fit);
			if (dist < min_value) min_value = dist;
		}
		IGD += min_value;
	}
	return IGD / size;
}


void process(double stop_time) {
// 	freopen("result.txt", "w", stdout);
	clock_t start, finish;
	start = clock();
	init_weight();
	init_pop(pop);
	double lambda[MAX], max_num[MAX], min_num[MAX]; 
	double invoke_num[MAX];
	int IGD_count = 1;
	int i = 0, j = 0, iter = 1;
//	freopen("lambda.txt", "w", stdout);
//	int count[MAX];
//	for (i = 0; i < weights_size; ++i) count[i] = 0;
	double prob[MAX];
	for (i = 0; i < weights_size; ++i) prob[i] = 1;
	while (true) {
		for (j = 0; j < weights_size; ++j) {
	//			printf("%d\n", iter);
			find_max_min(EP, max_num, min_num);
	 		int lambda_index = j;//wheel_routte(invoke_num, weights_size);
			generate_rand_lambda(lambda, radius, lambda_index);
	//		for (i = 0; i < obj_num; ++i) lambda[i] = weights[j].lambda[i];
			int index = find_best_solution(pop, lambda, max_num, min_num);
			local_search(pop[index], lambda, max_num, min_num);
			finish = clock();
			double now_time = (double)(finish - start) / CLOCKS_PER_SEC;

			//if (now_time > IGD_count * frac_time) {
			//	double IGD_num = IGD(EP, ref_p);
			//	IGD_time[IGD_count - 1] = now_time;
			//	IGD_value[IGD_count - 1] = IGD_num;
			//	++IGD_count;
			//}
			if ((double)((finish - start) / CLOCKS_PER_SEC) > stop_time)break;
		}
		finish = clock();
		if ((double)((finish - start) / CLOCKS_PER_SEC) > stop_time)break;
	}
	//output(EP);
	//freopen("oper.txt", "w", stdout);
	//for (i = 0; i < weights_size; ++i) {
	//	for (j = 0; j < oper_num; ++j) {
	//		printf("%d ", weights[i].invoke_num_oper[j]);
	//	}
	//	printf("\n");
	//}
}

bool update_best(Chromosome &new_chromosome) {

	int size = EP.size();
	if (total_best.size() == 0)total_best.push_back(new_chromosome);
	else {
		vector<Chromosome>::iterator iter = total_best.begin();
		for (; iter != total_best.end(); ++iter) {
			if (is_better(*iter, new_chromosome) || is_equal(*iter, new_chromosome)) return false;
		}
		iter = total_best.begin();
		while (iter != total_best.end()) {
			if (is_better(new_chromosome, *iter)) iter = total_best.erase(iter);
			else ++iter;
		}
		total_best.push_back(new_chromosome);
	}
	return true;

}

void output_best() {

	sort(total_best.begin(), total_best.end());
	vector<Chromosome>::iterator iter;
	for (iter = total_best.begin(); iter != total_best.end();++iter) {
		printf("%lf %lf %lf %lf %lf\n", iter->f[0], iter->f[1], iter->f[2], iter->f[3], iter->f[4]);
	}
	//	printf("total solutions: %d\n", total_best.size());

}
 
//int main() {
//	srand(time(NULL));
//	//char *spec_file = "C:\\Work\\programming\\VRP_source\\SDPPTW_source\\data\\SDPPTW_data\\SDPPTW_data\\test50-0-0-0-0.d0.tw0Specs_sdp.dat";
//	//char *distance_file = "C:\\Work\\programming\\VRP_source\\SDPPTW_source\\data\\SDPPTW_data\\SDPPTW_data\\test50-0-0-0-0.d0.tw0DistanceMatrix.dat";
//	//char *time_file = "C:\\Work\\programming\\VRP_source\\SDPPTW_source\\data\\SDPPTW_data\\SDPPTW_data\\test50-0-0-0-0.d0.tw0TimeMatrix.dat";
//	//cust_num = 50; obj_num = 5;
// //   getData(distance_file, time_file, spec_file);
//	char *distance_file = "C:\\Work\\programming\\VRP_source\\SDPPTW_source\\MOSDPPTW\\Project1\\large_data\\dataset1\\inputdistancetime_1_1601.txt";
//	char *spec_file = "C:\\Work\\programming\\VRP_source\\SDPPTW_source\\MOSDPPTW\\Project1\\large_data\\dataset1\\inputnode_1_1601.txt";
//	getData(distance_file, spec_file);
//	obj_num = 5;
//	process(600);
//	return 0;
//}

void load_ref(char *file) {
	freopen(file, "r", stdin);
	Chromosome ind; double ref_num; int count = 0;
	while (scanf("%lf", &ref_num) != EOF) {
		ind.f[count++] = ref_num;
		if (count == obj_num) {
			count = 0;
			ref_p.push_back(ind);
		}
	}
}


//////////////////for large data//////////////////////
int main(int argc, char **argv) {
	srand(time(NULL));
	if (argc<3) { printf("  specify data and output files");exit(1); }
	//	 freopen("result.txt","w",stdout);
	clock_t start, finish;
	char r_distance[100], r_spec[100];
	strcpy(r_distance, argv[1]);
	strcpy(r_spec, argv[2]);
	//strcpy(r_distance,"distance.txt");
	//strcpy(r_time,"time.txt");
	//strcpy(r_spec,"spec.txt");
	double stopTime = atof(argv[4]);
	//cust_num = atoi(argv[6]);
	//cust_num = 50;
	// 	double stopTime=117;
	getData(r_distance, r_spec);
	//double sum = 0;
	//for(int i = 0; i <= cust_num; ++i){
	//	sum += customer[i].d_demand - customer[i].p_demand;
	//}
	//printf("%lf\n", sum);
	//start=clock();
	int i;
	for (i = 0;i<1;++i) {
		char out_file[100];
		strcpy(out_file, argv[3]);
		strcat(out_file, "result");
		char num[3];
		sprintf(num, "%d", i);
		strcat(out_file, num);
		strcat(out_file, ".txt");
		freopen(out_file, "w", stdout);
		process(stopTime);
		output(EP);
		for (int j = 0;j<EP.size();++j) update_best(EP[j]);
		EP.clear();
	}
	//finish=clock();
	char out_file[100];
	strcpy(out_file, argv[3]);
	strcat(out_file, "final.txt");
	freopen(out_file, "w", stdout);
	output_best();
	//	printf("time: %lf\n",(double)((finish-start)/CLOCKS_PER_SEC)/i);
	return 0;
}
///////////////////////////////////////////////////////

//int main(int argc, char **argv) {
//	srand(time(NULL));
//	if (argc<6) { printf("  specify data and output files");exit(1); }
//	//	 freopen("result.txt","w",stdout);
//	clock_t start, finish;
//	char r_distance[100], r_time[100], r_spec[100];
//	strcpy(r_distance, argv[1]);
//	strcpy(r_time, argv[2]);
//	strcpy(r_spec, argv[3]);
//	//strcpy(r_distance,"distance.txt");
//	//strcpy(r_time,"time.txt");
//	//strcpy(r_spec,"spec.txt");
//	double stopTime = atof(argv[5]);
//	cust_num = atoi(argv[6]);
//	//cust_num = 50;
//	// 	double stopTime=117;
//	getData(r_distance, r_time, r_spec);
//	//double sum = 0;
//	//for(int i = 0; i <= cust_num; ++i){
//	//	sum += customer[i].d_demand - customer[i].p_demand;
//	//}
//	//printf("%lf\n", sum);
//	//start=clock();
//	int i;
//	for (i = 0;i<30;++i) {
//		char out_file[100];
//		strcpy(out_file, argv[4]);
//		strcat(out_file, "result");
//		char num[3];
//		sprintf(num, "%d", i);
//		strcat(out_file, num);
//		strcat(out_file, ".txt");
//		freopen(out_file, "w", stdout);
//		process(stopTime);
//		output(EP);
//		for (int j = 0;j<EP.size();++j) update_best(EP[j]);
//		EP.clear();
//	}
//	//finish=clock();
//	char out_file[100];
//	strcpy(out_file, argv[4]);
//	strcat(out_file, "final.txt");
//	freopen(out_file, "w", stdout);
//	output_best();
//	//	printf("time: %lf\n",(double)((finish-start)/CLOCKS_PER_SEC)/i);
//	return 0;
//}


//int main(int argc, char **argv) {
//	srand(time(NULL));
//	if (argc<2) exit(0);
//	char r_distance[100], r_time[100], r_spec[100];
//	strcpy(r_distance, argv[1]);
//	strcpy(r_spec, argv[2]);
//	//strcpy(r_distance,"distance.txt");
//	//strcpy(r_time,"time.txt");
//	//strcpy(r_spec,"spec.txt");
//	double stopTime = atof(argv[4]);
//	frac_time = stopTime / 10;
//	//cust_num = 50;
//	// 	double stopTime=117;
//	getData(r_distance, r_spec);
//	//double sum = 0;
//	load_ref(argv[5]);
//	//FILE *fp;
//	//    get_data("C:\\Work\\programming\\mubqp\\TS\\run\\mubqp_-0.48_3_5000_0.8_0.dat");
//	//	load_ref("C:\\Work\\programming\\EnsembleAlgorithm\\data\\archive\\5000\\3_-0.5_ref.txt");
//	for (int i = 0; i < 30; ++i) {
//		IGD_value[i] = 0;
//		IGD_time[i] = 0;
//	}
//	for (int i = 0; i < 1; ++i) {
//
//		//		freopen("result.txt", "w", stdout);
//		process(stopTime);
//
//		//	 	freopen("result.txt", "w", stdout);
//		freopen(argv[3], "w", stdout);
//
//		for (int i = 0; i < 10; ++i) {
//			printf("%lf %lf\n", IGD_time[i], IGD_value[i]);
//		}
//
//		//	output(EP);
//		//    fclose(fp);
//		//	fp = fopen(argv[3], "a");
//		//  freopen(argv[3], "a", stdout);
//		//  printf("%lf\n", hv);
//		//   fclose(fp);
//		//	fp = fopen(argv[4], "a");
//		//	freopen(argv[4], "a", stdout);
//		//	for (int j = 0; j < algo_num; ++j) printf("%d ", algo_count[j]);
//		// 	printf("\n");
//		//	EP.clear();
//	}
//	return 0;
//}
