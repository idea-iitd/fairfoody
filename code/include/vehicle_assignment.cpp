#include <bits/stdc++.h>
#include "global.hpp"
#include "graph_util.hpp"
#include "graph_util.hpp"
#include "hungarian.hpp"
#include "routeplan.hpp"
#include "dsu.hpp"
#include "hhl_query.hpp"
#include "food_data_util.hpp"
#include "vehicle_assignment.hpp"
#include "constants.hpp"

using namespace std;

// route_plan, plan_cost, delivery_times
typedef pair< pair<vector<event>, double>, unordered_map<string, double>> best_plan_tuple;

mt19937 vh_g(global_conf.random_seed);

// Return bearing angle between two lat/lons
// input in radians
double get_bearing(double lat, double lon, double lat2, double lon2){
    double teta1 = lat;
    double teta2 = lat2;
    double delta2 = lon2-lon;
    //==================Heading Formula Calculation================//
    double y = sin(delta2) * cos(teta2);
    double x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
    double brng = atan2(y,x);
    return brng;
}

// Direction angle between two lat/lons
// input in degrees
// output in radians
double get_direction(double lat, double lon, double lat2, double lon2){
    const long double PI = 3.141592653589793238L;
    lat = lat * PI/180.0;
    lon = lon * PI/180.0;
    lat2 = lat2 * PI/180.0;
    lon2 = lon2 * PI/180.0;
    return get_bearing(lat, lon, lat2, lon2);
}

// Return the heuristic function value
double heuristic_destination(vehicle &vh, long long int query_node){
    long long int dest_node = vh.route_plan[0].node;
    pair<double, double> vh_latlon = nodes_to_latlon[vh.get_current_location()];
    pair<double, double> ord_latlon = nodes_to_latlon[dest_node];
    pair<double, double> query_latlon = nodes_to_latlon[query_node];
    double dir1 = get_direction(vh_latlon.first, vh_latlon.second, ord_latlon.first, ord_latlon.second);
    double dir2 = get_direction(vh_latlon.first, vh_latlon.second, query_latlon.first, query_latlon.second);
    return -cos(dir1 - dir2);
}

// Return the heuristic function value used in order graph
double heuristic_destination_og(order &o, long long int query_node){
    long long int dest_node = o.customer.cust_node;
    pair<double, double> vh_latlon = nodes_to_latlon[o.restaurant.rest_node];
    pair<double, double> ord_latlon = nodes_to_latlon[dest_node];
    pair<double, double> query_latlon = nodes_to_latlon[query_node];
    double dir1 = get_direction(vh_latlon.first, vh_latlon.second, ord_latlon.first, ord_latlon.second);
    double dir2 = get_direction(vh_latlon.first, vh_latlon.second, query_latlon.first, query_latlon.second);
    return -cos(dir1 - dir2);
}

// Normalized heuristic function value
// low is good
double heuristic_function(vehicle &vh, long long int &query_node){
    if(vh.route_plan.empty())
        return 0.0;
    return ((1.0+heuristic_destination(vh, query_node))/2.0);
}

// Normalized heuristic function value for order graph
double heuristic_function_og(order &o, long long int &query_node){
    return ((1.0+heuristic_destination_og(o, query_node))/2.0);
}

// Assuming a empty vehicle at routeplan start, 
// check if capacity constraint is satisfied
bool routeplan_capacity_constraint(vector<event> &route_plan){
    long long int curr_load = 0;
    for (int i = 0; i < int(route_plan.size()); i++){
        // If Pickup, increase item count
        if (route_plan[i].type == 0){
            curr_load += route_plan[i].order_obj.items;
            if (curr_load > global_conf.vehicle_max_items)
                return false;
        }
        // If Delivery, decrease item count
        else
            curr_load -= route_plan[i].order_obj.items;
    }
    return true;
}

// Assuming a empty vehicle at routeplan start, 
// find delivered times of orders
unordered_map<string, double> get_routeplan_delivered_times(vector<event> &route_plan, double global_time){
    unordered_map<string, double> delivered_time;
    long long int curr_node = route_plan[0].node;
    for (int i = 0; i < int(route_plan.size()); i++){
        order event_order = route_plan[i].order_obj;
        double time_taken = hhl_sp_query(curr_node, route_plan[i].node);
        if (time_taken + FP_EPSILON >= MAX_NUM){
            // if any two event nodes in route plan were not reachable, return empty delivered times
            return {};
        }
        global_time += time_taken;
        // Pick Up
        if (route_plan[i].type == 0){
            double food_prep_time = event_order.order_time + event_order.prep_time;
            global_time = max(global_time, food_prep_time);
        }
        else
            delivered_time[event_order.order_id] = global_time;
        curr_node = route_plan[i].node;
    }
    return delivered_time;
}

// Assuming a empty vehicle at routeplan start, 
// find routeplan XDT
best_plan_tuple get_routeplan_cost(vector<event> &rp, double global_time){
    if (!routeplan_capacity_constraint(rp))
        return {{rp, MAX_NUM}, {}};
    unordered_map<string, double> d_times = get_routeplan_delivered_times(rp, global_time);
    if (d_times.empty())
        return {{rp, MAX_NUM}, {}};
    return {{rp, get_route_plan_extra_delivery_time(d_times, rp)}, d_times};
    return {{rp, MAX_NUM}, {}};
}

// Generate the order graph
vector<vector<best_plan_tuple>> generate_start_order_undirected_graph(vector<order> order_set, double global_time){
    int n = order_set.size();
    vector<vector<best_plan_tuple>> order_graph(n, vector<best_plan_tuple>(n, {{{}, MAX_NUM}, {}}));
    for(int i = 0; i < n; i++){
        order order_i = order_set[i];
        event event_i_p(order_i, 0);
        event event_i_d(order_i, 1);
        vector<event> temp_e({event_i_p, event_i_d});
        best_plan_tuple temp = get_routeplan_cost(temp_e, global_time);
        order_graph[i][i] = temp;
    }
    for (int i = 0; i < n; i++){
        order order_i = order_set[i];
        event event_i_p(order_i, 0);
        event event_i_d(order_i, 1);
        for(int j = i+1; j < n; j++){
            order order_j = order_set[j];
            event event_j_p(order_j, 0);
            event event_j_d(order_j, 1);
            double sp_ij = hhl_sp_query(event_i_p.node, event_j_p.node);
            double sp_ji = hhl_sp_query(event_j_p.node, event_i_p.node);
            if (sp_ij >= vehicle_rest_radius_cap || sp_ji >= vehicle_rest_radius_cap)
                continue;
            vector<event> rp_1{event_i_p, event_i_d, event_j_p, event_j_d};
            vector<event> rp_2{event_i_p, event_j_p, event_i_d, event_j_d};
            vector<event> rp_3{event_i_p, event_j_p, event_j_d, event_i_d};
            vector<event> rp_4{event_j_p, event_j_d, event_i_p, event_i_d};
            vector<event> rp_5{event_j_p, event_i_p, event_j_d, event_i_d};
            vector<event> rp_6{event_j_p, event_i_p, event_i_d, event_j_d};
            vector<vector<event>> all_rp{rp_1, rp_2, rp_3, rp_4, rp_5, rp_6};
            best_plan_tuple bp{{{}, MAX_NUM}, {}};
            for(auto rp:all_rp){
                best_plan_tuple cp = get_routeplan_cost(rp, global_time);
                if (cp.first.second < bp.first.second){
                    bp = cp;
                }
            }
            if (bp.first.second + FP_EPSILON >= MAX_NUM)
                continue;
            order_graph[i][j] = bp;
            order_graph[i][j].first.second = order_graph[i][j].first.second - order_graph[i][i].first.second - order_graph[j][j].first.second;
        }
    }
    return order_graph;
}

// Generate the order graph with best first search
vector<vector<best_plan_tuple>> generate_start_order_undirected_graph_vh(vector<order> order_set, double global_time){
    int curr_time_slot = ((((long long int)(global_time))%86400 + 86400)%86400)/3600;
    int n = order_set.size();
    vector<vector<best_plan_tuple>> order_graph(n, vector<best_plan_tuple>(n, {{{}, MAX_NUM}, {}}));
    for(int i = 0; i < n; i++){
        order order_i = order_set[i];
        event event_i_p(order_i, 0);
        event event_i_d(order_i, 1);
        vector<event> temp_e({event_i_p, event_i_d});
        best_plan_tuple temp = get_routeplan_cost(temp_e, global_time);
        order_graph[i][i] = temp;
    }
    vector<vector<int>> node_to_order(nodes_to_latlon.size());
    for (int i = 0; i < n; i++){
        node_to_order[order_graph[i][i].first.first[0].node].push_back(i);
    }
    int max_count_edges = max(1, vehicle_explore_frac);
    double max_time_dist = vehicle_explore_rad;
    vector<bool> visited(nodes_to_latlon.size(), false);
    vector<double> dist_vh(nodes_to_latlon.size(), MAX_NUM);
    for (int i = 0; i < n; i++){
        order order_i = order_set[i];
        event event_i_p(order_i, 0);
        event event_i_d(order_i, 1);
        int count_edges = 0;
        fill(visited.begin(), visited.end(), false);
        fill(dist_vh.begin(), dist_vh.end(), MAX_NUM);
        long long int vh_start_node = event_i_p.node;
        dist_vh[vh_start_node] = 0.0;
        priority_queue<pair<double, long long int>> pq;
        pq.push({-dist_vh[vh_start_node], vh_start_node});
        double dist, alt;
        long long int u, v;
        while(!pq.empty()){
            pair<double, long long int> top = pq.top();
            pq.pop();
            u = top.second;
            if (visited[u])
                continue;
            if (count_edges > max_count_edges || dist_vh[u] > 1.0)
                break;
            visited[u] = true;
            if (node_to_order[u].size() != 0){
                for(auto j : node_to_order[u]){
                    if (j == i)
                        continue;
                    if (count_edges > max_count_edges)
                        break;
                    order order_j = order_set[j];
                    event event_j_p(order_j, 0);
                    event event_j_d(order_j, 1);
                    vector<event> rp_1{event_i_p, event_i_d, event_j_p, event_j_d};
                    vector<event> rp_2{event_i_p, event_j_p, event_i_d, event_j_d};
                    vector<event> rp_3{event_i_p, event_j_p, event_j_d, event_i_d};
                    vector<vector<event>> all_rp{rp_1, rp_2, rp_3};
                    best_plan_tuple bp{{{}, MAX_NUM}, {}};
                    for(auto rp:all_rp){
                        best_plan_tuple cp = get_routeplan_cost(rp, global_time);
                        if (cp.first.second < bp.first.second){
                            bp = cp;
                        }
                    }
                    int og_i = min(i,j);
                    int og_j = max(i,j);
                    if (bp.first.second < order_graph[og_i][og_j].first.second){
                        order_graph[og_i][og_j] = bp;
                        count_edges++;
                    }
                }
            }
            if (count_edges > max_count_edges)
                break;
            for(int ei = 0; ei < int(edges[u].size()); ei++){
                v = edges[u][ei];
                double e_weight = get_edge_weight(u, v, curr_time_slot);
                alt = dist_vh[u] + e_weight/max_time_dist;
                if (alt < dist_vh[v]){
                    dist_vh[v] = alt;
                    pq.push(make_pair(-alt, v));
                }
            }
        }
    }
    for (int i = 0; i < n; i++){
        for(int j = i+1; j < n; j++){
            if (order_graph[i][j].first.second + FP_EPSILON >= MAX_NUM)
                continue;
            order_graph[i][j].first.second = order_graph[i][j].first.second - order_graph[i][i].first.second - order_graph[j][j].first.second;
        }
    }
    return order_graph;
}

// Generate the order graph with best first search and angular distance
vector<vector<best_plan_tuple>> generate_start_order_undirected_graph_as(vector<order> order_set, double global_time){
    int curr_time_slot = ((((long long int)(global_time))%86400 + 86400)%86400)/3600;
    int n = order_set.size();
    vector<vector<best_plan_tuple>> order_graph(n, vector<best_plan_tuple>(n, {{{}, MAX_NUM}, {}}));
    for(int i = 0; i < n; i++){
        order order_i = order_set[i];
        event event_i_p(order_i, 0);
        event event_i_d(order_i, 1);
        vector<event> temp_e({event_i_p, event_i_d});
        best_plan_tuple temp = get_routeplan_cost(temp_e, global_time);
        order_graph[i][i] = temp;
    }
    vector<vector<int>> node_to_order(nodes_to_latlon.size());
    for (int i = 0; i < n; i++){
        node_to_order[order_graph[i][i].first.first[0].node].push_back(i);
    }
    // n vehicles, n orders
    int max_count_edges = max(1, vehicle_explore_frac);
    double max_time_dist = slotted_max_time[curr_time_slot];
    vector<bool> visited(nodes_to_latlon.size(), false);
    vector<double> dist_vh(nodes_to_latlon.size(), MAX_NUM);
    vector<double> dist_ac(nodes_to_latlon.size(), MAX_NUM);
    for (int i = 0; i < n; i++){
        order order_i = order_set[i];
        event event_i_p(order_i, 0);
        event event_i_d(order_i, 1);
        int count_edges = 0;
        fill(visited.begin(), visited.end(), false);
        fill(dist_vh.begin(), dist_vh.end(), MAX_NUM);
        fill(dist_ac.begin(), dist_ac.end(), MAX_NUM);
        long long int vh_start_node = event_i_p.node;
        dist_vh[vh_start_node] = 0.0;
        dist_ac[vh_start_node] = 0.0;
        priority_queue<pair<double, long long int>> pq;
        pq.push({-dist_vh[vh_start_node], vh_start_node});
        double dist, alt;
        long long int u, v;
        while(!pq.empty()){
            pair<double, long long int> top = pq.top();
            pq.pop();
            u = top.second;
            if (visited[u])
                continue;
            if (count_edges > max_count_edges)
                break;
            if (dist_ac[u] > vehicle_rest_radius_cap){
                if (hhl_sp_query(vh_start_node, u) > vehicle_rest_radius_cap)
                    break;
            }
            visited[u] = true;
            if (node_to_order[u].size() != 0){
                for(auto j : node_to_order[u]){
                    if (j == i)
                        continue;
                    if (count_edges > max_count_edges)
                        break;
                    order order_j = order_set[j];
                    event event_j_p(order_j, 0);
                    event event_j_d(order_j, 1);
                    vector<event> rp_1{event_i_p, event_i_d, event_j_p, event_j_d};
                    vector<event> rp_2{event_i_p, event_j_p, event_i_d, event_j_d};
                    vector<event> rp_3{event_i_p, event_j_p, event_j_d, event_i_d};
                    vector<vector<event>> all_rp{rp_1, rp_2, rp_3};
                    best_plan_tuple bp{{{}, MAX_NUM}, {}};
                    for(auto rp:all_rp){
                        best_plan_tuple cp = get_routeplan_cost(rp, global_time);
                        if (cp.first.second < bp.first.second){
                            bp = cp;
                        }
                    }
                    int og_i = min(i,j);
                    int og_j = max(i,j);
                    if (bp.first.second < order_graph[og_i][og_j].first.second){
                        order_graph[og_i][og_j] = bp;
                        count_edges++;
                    }
                }
            }
            if (count_edges > max_count_edges)
                break;
            for(int ei = 0; ei < int(edges[u].size()); ei++){
                v = edges[u][ei];
                double e_weight = get_edge_weight(u, v, curr_time_slot);
                double heur_val = heuristic_function_og(order_i, v);
                alt = dist_vh[u] + (1-heuristic_multiplier)*(e_weight)/max_time_dist + heuristic_multiplier*heur_val;
                if (alt < dist_vh[v]){
                    dist_ac[v] = dist_ac[u]+e_weight;
                    dist_vh[v] = alt;
                    pq.push(make_pair(-alt, v));
                }
            }
        }
    }
    for (int i = 0; i < n; i++){
        for(int j = i+1; j < n; j++){
            if (order_graph[i][j].first.second + FP_EPSILON >= MAX_NUM)
                continue;
            order_graph[i][j].first.second = order_graph[i][j].first.second - order_graph[i][i].first.second - order_graph[j][j].first.second;
        }
    }
    return order_graph;
}

vector<vector<best_plan_tuple>> generate_start_order_undirected_graph_sp_as(vector<order> order_set, double global_time){
	cout << "OG ST" << endl;
    int curr_time_slot = ((((long long int)(global_time))%86400 + 86400)%86400)/3600;
	int n = order_set.size();
	vector<vector<best_plan_tuple>> order_graph(n, vector<best_plan_tuple>(n, {{{}, MAX_NUM}, {}}));

	for(int i = 0; i < n; i++){
		order order_i = order_set[i];
		
		event event_i_p(order_i, 0);
		event event_i_d(order_i, 1);

		vector<event> temp_e({event_i_p, event_i_d});
		best_plan_tuple temp = get_routeplan_cost(temp_e, global_time);
		order_graph[i][i] = temp;
	}

	vector<vector<int>> node_to_order(nodes_to_latlon.size());
	for (int i = 0; i < n; i++){
		node_to_order[order_graph[i][i].first.first[0].node].push_back(i);
	}

	//int max_count_edges = max(1, n/vehicle_explore_frac);
	// n vehicles, n orders
	int max_count_edges = max(1, vehicle_explore_frac);
	//int max_count_edges = max(1, n/vehicle_explore_frac);
	// double max_time_dist = vehicle_explore_rad;
	double max_time_dist = slotted_max_time[curr_time_slot];

	vector<bool> visited(nodes_to_latlon.size(), false);
	vector<double> dist_vh(nodes_to_latlon.size(), MAX_NUM);
	vector<double> dist_ac(nodes_to_latlon.size(), MAX_NUM);

	for (int i = 0; i < n; i++){
		
		order order_i = order_set[i];
		
		event event_i_p(order_i, 0);
		event event_i_d(order_i, 1);

		int count_edges = 0;

		fill(visited.begin(), visited.end(), false);
		fill(dist_vh.begin(), dist_vh.end(), MAX_NUM);
		fill(dist_ac.begin(), dist_ac.end(), MAX_NUM);

		long long int vh_start_node = event_i_p.node;
		dist_vh[vh_start_node] = 0.0;
		dist_ac[vh_start_node] = 0.0;
		priority_queue<pair<double, long long int>> pq;

		pq.push({-dist_vh[vh_start_node], vh_start_node});
		double dist, alt;
		long long int u, v;

		while(!pq.empty()){
			pair<double, long long int> top = pq.top();
			pq.pop();

			u = top.second;

			// cout << "EX " << u << endl;

			if (visited[u])
				continue;

			if (count_edges > max_count_edges)
				break;

			if (dist_ac[u] > vehicle_rest_radius_cap){
				// dist_ac[u] >= shortest_path
				if (hhl_sp_query(vh_start_node, u) > vehicle_rest_radius_cap)
					break;
			}


			visited[u] = true;
			if (node_to_order[u].size() != 0){
				// if (hhl_sp_query(vh_start_node, u) < vehicle_rest_radius_cap){
					for(auto j : node_to_order[u]){
						if (j == i)
							continue;
						
						if (count_edges > max_count_edges)
							break;

						order order_j = order_set[j];

						event event_j_p(order_j, 0);
						event event_j_d(order_j, 1);

						vector<event> rp_1{event_i_p, event_i_d, event_j_p, event_j_d};
						vector<event> rp_2{event_i_p, event_j_p, event_i_d, event_j_d};
						vector<event> rp_3{event_i_p, event_j_p, event_j_d, event_i_d};
						
						vector<vector<event>> all_rp{rp_1, rp_2, rp_3};

						best_plan_tuple bp{{{}, MAX_NUM}, {}}; 
						for(auto rp:all_rp){
							best_plan_tuple cp = get_routeplan_cost(rp, global_time);
							if (cp.first.second < bp.first.second){
								bp = cp;
							}
						}
						
						int og_i = min(i,j);
						int og_j = max(i,j);
						if (bp.first.second < order_graph[og_i][og_j].first.second){
							order_graph[og_i][og_j] = bp;
							count_edges++;
						}
					}
				// }
			}

			if (count_edges > max_count_edges)
				break;

			for(int ei = 0; ei < int(edges[u].size()); ei++){
				v = edges[u][ei];
				double e_weight = get_edge_weight(u, v, curr_time_slot);
				double heur_val = heuristic_function_og(order_i, v);

				alt = dist_vh[u] + (1-heuristic_multiplier)*(e_weight)/max_time_dist + heuristic_multiplier*heur_val;
                // alt = (1-heuristic_multiplier)*(dist_ac[u]+e_weight)/max_time_dist + heuristic_multiplier*heur_val;
				//alt = (1-heuristic_multiplier)*(hhl_sp_query(u,v))/max_time_dist + heuristic_multiplier*heur_val;
			

				if (alt < dist_vh[v]){
					dist_ac[v] = dist_ac[u]+e_weight;
					dist_vh[v] = alt;
					pq.push(make_pair(-alt, v));
				}
			}
		}
	}

	
	for (int i = 0; i < n; i++){
		for(int j = i+1; j < n; j++){
			if (order_graph[i][j].first.second + FP_EPSILON >= MAX_NUM)
				continue;
			order_graph[i][j].first.second = order_graph[i][j].first.second - order_graph[i][i].first.second - order_graph[j][j].first.second;
			if (order_graph[i][j].first.second + FP_EPSILON < 0){
				cout <<  order_graph[i][j].first.second << " "  << order_graph[i][i].first.second << " " << order_graph[j][j].first.second << endl; 
				for (auto it:order_graph[i][j].first.first)
					cout << it.str_val() << "|";
				cout << "ERROR FOUND" << endl;
				throw "ERROR FOUND \n";
			}
		}
	}

	return order_graph;
}

// Given two routeplans, returns a interleaving routeplan with minimal cost
// while keeping the sequence of input routeplans fixed
best_plan_tuple get_inserted_bestplan(vector<event> &rp0, vector<event> &rp1, double global_time){
    int n0 = rp0.size();
    int n1 = rp1.size();
    vector<int> perm(n0 + n1, 0);
    for(int i = n0; i < n0+n1; i++){
        perm[i] = 1;
    }
    best_plan_tuple bp{{{}, MAX_NUM}, {}};
    vector<event> new_rp(n0 + n1);
    do {
        int cnt0=0;
        int cnt1=0;
        for(auto it:perm){
            if (it == 0){
                new_rp[cnt1+cnt0] = rp0[cnt0];
                cnt0++;
            }
            else{
                new_rp[cnt1+cnt0] = rp1[cnt1];
                cnt1++;
            }
        }
        best_plan_tuple cp = get_routeplan_cost(new_rp, global_time);
        if (cp.first.second < bp.first.second){
            bp = cp;
        }
    } while (next_permutation(perm.begin(),perm.end()));
    return bp;
}

// Perform clustering on order graph
vector<best_plan_tuple> hac_cluster_orders(vector<order> &order_set, double global_time){
    // merge_parameter
    double max_merge_cost = max_merge_cost_edt;
    int n = order_set.size();
    if (n == 0)
        return {};
    DSU dsu;
    dsu.init(n);
    vector<vector<best_plan_tuple>> order_graph = generate_start_order_undirected_graph(order_set, global_time);
    // Reference - https://nlp.stanford.edu/IR-book/pdf/17hier.pdf
    double double_to_int = 100000000.0;
    double total_sum = 0.0;
    double total_batches = n;
    map<pair<long long int, int>, pair<int, int>> PQ[n];
    for(int i = 0; i < n; i++){
        for(int j = i+1; j < n; j++){
            double cost = order_graph[i][j].first.second;
            if (cost + FP_EPSILON >= MAX_NUM)
                continue;
            PQ[i][{(long long int)(cost*double_to_int), j}] = {i,j};
        }
        if (order_graph[i][i].first.second + FP_EPSILON <= MAX_NUM)
            total_sum += order_graph[i][i].first.second;
    }
    vector<int> I(n, 1);
    int iter_c = 0;
    while(true){
        if (total_sum/total_batches > max_merge_cost)
            break;
        long long int curr_min = (long long int)((max_merge_cost+1)*double_to_int);
        int i_min = -1, j_min = -1;
        for(int i = 0; i < n; i++){
            if ((I[i] == 0) || PQ[i].empty())
                continue;
            pair<long long int, int> min_elem = PQ[i].begin()->first;
            if (min_elem.first < curr_min){
                curr_min = min_elem.first;
                i_min = i;
                j_min = min_elem.second;
            }
        }
        if (i_min == -1 || j_min == -1)
            break;
        dsu.merge(i_min, j_min);
        int set_i = dsu.root(i_min);
        if (set_i != i_min)
            throw "PROBLEM IN HAC \n";
        // order_graph[set_i][set_i] = get_routeplan_cost(order_graph[i_min][j_min].first.first, global_time);
        double new_cost = order_graph[i_min][j_min].first.second + order_graph[i_min][i_min].first.second + order_graph[j_min][j_min].first.second;
        order_graph[set_i][set_i] = order_graph[i_min][j_min];
        order_graph[set_i][set_i].first.second = new_cost;
        total_sum += order_graph[i_min][j_min].first.second;
        total_batches--;
        I[j_min] = 0;
        PQ[i_min].clear();
        for(int i = 0; i < j_min; i++){
            if (I[i] == 1){
                double cost = order_graph[i][j_min].first.second;
                PQ[i].erase({(long long int)(cost*double_to_int), j_min});
            }
        }
        for(int i = 0; i < i_min; i++){
            if (I[i] == 1){
                double cost = order_graph[i][i_min].first.second;
                PQ[i].erase({(long long int)(cost*double_to_int), i_min});
            }
        }
        int size_set_i = dsu.set_to_elems[set_i].size();

        for(int i = 0; i < i_min; i++){
            if (I[i] == 1){
                int size_i = dsu.get_elems(i).size();
                if (size_set_i + size_i > batching_cap ||
                    (order_graph[i][i_min].first.second + FP_EPSILON >= MAX_NUM) ||
                    (order_graph[i][j_min].first.second + FP_EPSILON >= MAX_NUM)){
                        continue;
                }
                order_graph[i][set_i] = get_inserted_bestplan(order_graph[set_i][set_i].first.first,
                                                              order_graph[i][i].first.first, global_time);
                order_graph[i][set_i].first.second -= (order_graph[set_i][set_i].first.second + order_graph[i][i].first.second);
                double cost = order_graph[i][set_i].first.second;
                if (cost + FP_EPSILON >= MAX_NUM)
                    continue;
                PQ[i][{(long long int)(cost*double_to_int), set_i}] = {i,set_i};
            }
        }
        for(int i = set_i+1; i < n; i++){
            if (I[i] == 1){
                int size_i = dsu.get_elems(i).size();
                if (size_set_i + size_i > batching_cap ||
                    (order_graph[i_min][i].first.second + FP_EPSILON >= MAX_NUM) ||
                    (order_graph[i][j_min].first.second + FP_EPSILON >= MAX_NUM && order_graph[j_min][i].first.second + FP_EPSILON >= MAX_NUM)){
                        continue;
                }
                order_graph[set_i][i] = get_inserted_bestplan(order_graph[set_i][set_i].first.first,
                                                              order_graph[i][i].first.first, global_time);
                order_graph[set_i][i].first.second -= (order_graph[set_i][set_i].first.second + order_graph[i][i].first.second);
                double cost = order_graph[set_i][i].first.second;
                if (cost + FP_EPSILON > MAX_NUM)
                    continue;
                PQ[set_i][{(long long int)(cost*double_to_int), i}] = {set_i,i};
            }
        }
        iter_c++;
    }
    vector<best_plan_tuple> cluster_pack;
    for(auto it:dsu.set_to_elems){
        cluster_pack.push_back(order_graph[it.first][it.first]);
    }
    return cluster_pack;
}

// Perform clustering on order graph BFS
vector<best_plan_tuple> hac_cluster_orders_vh(vector<order> &order_set, double global_time){
    double max_merge_cost = max_merge_cost_edt;
    int n = order_set.size();
    if (n == 0)
        return {};
    DSU dsu;
    dsu.init(n);
    vector<vector<best_plan_tuple>> order_graph = generate_start_order_undirected_graph_vh(order_set, global_time);
    // Reference - https://nlp.stanford.edu/IR-book/pdf/17hier.pdf
    double double_to_int = 100000000.0;
    double total_sum = 0.0;
    double total_batches = n;
    map<pair<long long int, int>, pair<int, int>> PQ[n];
    for(int i = 0; i < n; i++){
        for(int j = i+1; j < n; j++){
            double cost = order_graph[i][j].first.second;
            if (cost + FP_EPSILON >= MAX_NUM)
                continue;
            PQ[i][{(long long int)(cost*double_to_int), j}] = {i,j};
        }
        if (order_graph[i][i].first.second + FP_EPSILON < MAX_NUM)
            total_sum += order_graph[i][i].first.second;
    }
    vector<int> I(n, 1);
    int iter_c = 0;
    while(true){
        if (total_sum/total_batches > max_merge_cost)
            break;
        long long int curr_min = (long long int)((max_merge_cost+1)*double_to_int);
        int i_min = -1, j_min = -1;
        for(int i = 0; i < n; i++){
            if ((I[i] == 0) || PQ[i].empty())
                continue;
            pair<long long int, int> min_elem = PQ[i].begin()->first;
            if (min_elem.first < curr_min){
                curr_min = min_elem.first;
                i_min = i;
                j_min = min_elem.second;
            }
        }
        if (i_min == -1 || j_min == -1)
            break;
        dsu.merge(i_min, j_min);
        int set_i = dsu.root(i_min);
        if (set_i != i_min)
            throw "PROBLEM IN HAC \n";
        // order_graph[set_i][set_i] = get_routeplan_cost(order_graph[i_min][j_min].first.first, global_time);
        double new_cost = order_graph[i_min][j_min].first.second + order_graph[i_min][i_min].first.second + order_graph[j_min][j_min].first.second;
        order_graph[set_i][set_i] = order_graph[i_min][j_min];
        order_graph[set_i][set_i].first.second = new_cost;
        total_sum += order_graph[i_min][j_min].first.second;
        total_batches--;
        I[j_min] = 0;
        PQ[i_min].clear();
        for(int i = 0; i < j_min; i++){
            if (I[i] == 1){
                double cost = order_graph[i][j_min].first.second;
                PQ[i].erase({(long long int)(cost*double_to_int), j_min});
            }
        }
        for(int i = 0; i < i_min; i++){
            if (I[i] == 1){
                double cost = order_graph[i][i_min].first.second;
                PQ[i].erase({(long long int)(cost*double_to_int), i_min});
            }
        }
        int size_set_i = dsu.set_to_elems[set_i].size();
        for(int i = 0; i < i_min; i++){
            if (I[i] == 1){
                int size_i = dsu.get_elems(i).size();
                if (size_set_i + size_i > batching_cap ||
                    (order_graph[i][i_min].first.second + FP_EPSILON >= MAX_NUM) ||
                    (order_graph[i][j_min].first.second + FP_EPSILON >= MAX_NUM)){
                        continue;
                }
                order_graph[i][set_i] = get_inserted_bestplan(order_graph[set_i][set_i].first.first,
                                                              order_graph[i][i].first.first, global_time);
                order_graph[i][set_i].first.second -= (order_graph[set_i][set_i].first.second + order_graph[i][i].first.second);
                double cost = order_graph[i][set_i].first.second;
                if (cost + FP_EPSILON >= MAX_NUM)
                    continue;
                PQ[i][{(long long int)(cost*double_to_int), set_i}] = {i,set_i};
            }
        }
        for(int i = set_i+1; i < n; i++){
            if (I[i] == 1){
                int size_i = dsu.get_elems(i).size();
                if (size_set_i + size_i > batching_cap ||
                    (order_graph[i_min][i].first.second + FP_EPSILON >= MAX_NUM) ||
                    (order_graph[i][j_min].first.second + FP_EPSILON >= MAX_NUM && order_graph[j_min][i].first.second + FP_EPSILON >= MAX_NUM)){
                        continue;
                }
                order_graph[set_i][i] = get_inserted_bestplan(order_graph[set_i][set_i].first.first,
                                                              order_graph[i][i].first.first, global_time);
                order_graph[set_i][i].first.second -= (order_graph[set_i][set_i].first.second + order_graph[i][i].first.second);
                double cost = order_graph[set_i][i].first.second;
                if (cost + FP_EPSILON >= MAX_NUM)
                    continue;
                PQ[set_i][{(long long int)(cost*double_to_int), i}] = {set_i,i};
            }
        }
        iter_c++;
    }
    vector<best_plan_tuple> cluster_pack;
    for(auto it:dsu.set_to_elems){
        cluster_pack.push_back(order_graph[it.first][it.first]);
    }
    return cluster_pack;
}

// Perform clustering on order graph BFS + Angular distance
vector<best_plan_tuple> hac_cluster_orders_as(vector<order> &order_set, double global_time){
    double max_merge_cost = max_merge_cost_edt;
    int n = order_set.size();
    if (n == 0)
        return {};
    DSU dsu;
    dsu.init(n);
    vector<vector<best_plan_tuple>> order_graph = generate_start_order_undirected_graph_as(order_set, global_time);
    // Reference - https://nlp.stanford.edu/IR-book/pdf/17hier.pdf
    double double_to_int = 100000000.0;
    double total_sum = 0.0;
    double total_batches = n;
    map<pair<long long int, int>, pair<int, int>> PQ[n];
    for(int i = 0; i < n; i++){
        for(int j = i+1; j < n; j++){
            double cost = order_graph[i][j].first.second;
            if (cost + FP_EPSILON >= MAX_NUM)
                continue;
            PQ[i][{(long long int)(cost*double_to_int), j}] = {i,j};
        }
        if (order_graph[i][i].first.second + FP_EPSILON < MAX_NUM)
            total_sum += order_graph[i][i].first.second;
    }
    vector<int> I(n, 1);
    int iter_c = 0;
    while(true){
        if (total_sum/total_batches > max_merge_cost)
            break;
        long long int curr_min = (long long int)((max_merge_cost+1)*double_to_int);
        int i_min = -1, j_min = -1;
        for(int i = 0; i < n; i++){
            if ((I[i] == 0) || PQ[i].empty())
                continue;
            pair<long long int, int> min_elem = PQ[i].begin()->first;
            if (min_elem.first < curr_min){
                curr_min = min_elem.first;
                i_min = i;
                j_min = min_elem.second;
            }
        }
        if (i_min == -1 || j_min == -1)
            break;
        dsu.merge(i_min, j_min);
        int set_i = dsu.root(i_min);
        if (set_i != i_min)
            throw "PROBLEM IN HAC \n";
        // order_graph[set_i][set_i] = get_routeplan_cost(order_graph[i_min][j_min].first.first, global_time);
        double new_cost = order_graph[i_min][j_min].first.second + order_graph[i_min][i_min].first.second + order_graph[j_min][j_min].first.second;
        order_graph[set_i][set_i] = order_graph[i_min][j_min];
        order_graph[set_i][set_i].first.second = new_cost;
        total_sum += order_graph[i_min][j_min].first.second;
        total_batches--;
        I[j_min] = 0;
        PQ[i_min].clear();
        for(int i = 0; i < j_min; i++){
            if (I[i] == 1){
                double cost = order_graph[i][j_min].first.second;
                PQ[i].erase({(long long int)(cost*double_to_int), j_min});
            }
        }
        for(int i = 0; i < i_min; i++){
            if (I[i] == 1){
                double cost = order_graph[i][i_min].first.second;
                PQ[i].erase({(long long int)(cost*double_to_int), i_min});
            }
        }
        int size_set_i = dsu.set_to_elems[set_i].size();
        for(int i = 0; i < i_min; i++){
            if (I[i] == 1){
                int size_i = dsu.get_elems(i).size();
                if (size_set_i + size_i > batching_cap ||
                    (order_graph[i][i_min].first.second + FP_EPSILON >= MAX_NUM) ||
                    (order_graph[i][j_min].first.second + FP_EPSILON >= MAX_NUM)){
                        continue;
                }
                order_graph[i][set_i] = get_inserted_bestplan(order_graph[set_i][set_i].first.first,
                                                              order_graph[i][i].first.first, global_time);
                order_graph[i][set_i].first.second -= (order_graph[set_i][set_i].first.second + order_graph[i][i].first.second);
                double cost = order_graph[i][set_i].first.second;
                if (cost + FP_EPSILON >= MAX_NUM)
                    continue;
                PQ[i][{(long long int)(cost*double_to_int), set_i}] = {i,set_i};
            }
        }
        for(int i = set_i+1; i < n; i++){
            if (I[i] == 1){
                int size_i = dsu.get_elems(i).size();
                if (size_set_i + size_i > batching_cap ||
                    (order_graph[i_min][i].first.second + FP_EPSILON >= MAX_NUM) ||
                    (order_graph[i][j_min].first.second + FP_EPSILON >= MAX_NUM && order_graph[j_min][i].first.second + FP_EPSILON >= MAX_NUM)){
                        continue;
                }
                order_graph[set_i][i] = get_inserted_bestplan(order_graph[set_i][set_i].first.first,
                                                              order_graph[i][i].first.first, global_time);
                order_graph[set_i][i].first.second -= (order_graph[set_i][set_i].first.second + order_graph[i][i].first.second);
                double cost = order_graph[set_i][i].first.second;
                if (cost + FP_EPSILON >= MAX_NUM)
                    continue;
                PQ[set_i][{(long long int)(cost*double_to_int), i}] = {set_i,i};
            }
        }
        iter_c++;
    }
    vector<best_plan_tuple> cluster_pack;
    for(auto it:dsu.set_to_elems){
        cluster_pack.push_back(order_graph[it.first][it.first]);
    }
    return cluster_pack;
}


// Perform clustering on order graph BFS + Angular distance but clustering restricted on number of vehicals
vector<best_plan_tuple> hac_cluster_orders_as_restrict(vector<order> &order_set, double global_time, int vh_size){
    double max_merge_cost = max_merge_cost_edt;
    int n = order_set.size();
    if (n == 0)
        return {};
    DSU dsu;
    dsu.init(n);
    vector<vector<best_plan_tuple>> order_graph = generate_start_order_undirected_graph_as(order_set, global_time);
    // Reference - https://nlp.stanford.edu/IR-book/pdf/17hier.pdf
    double double_to_int = 100000000.0;
    double total_sum = 0.0;
    double total_batches = n;
    map<pair<long long int, int>, pair<int, int>> PQ[n];
    for(int i = 0; i < n; i++){
        for(int j = i+1; j < n; j++){
            double cost = order_graph[i][j].first.second;
            if (cost + FP_EPSILON >= MAX_NUM)
                continue;
            PQ[i][{(long long int)(cost*double_to_int), j}] = {i,j};
        }
        if (order_graph[i][i].first.second + FP_EPSILON < MAX_NUM)
            total_sum += order_graph[i][i].first.second;
    }
    vector<int> I(n, 1);
    int iter_c = 0;
    while(true){
        if (total_sum/total_batches > max_merge_cost || total_batches <= vh_size * cluster_pct_frac)
            break;
        long long int curr_min = (long long int)((max_merge_cost+1)*double_to_int);
        int i_min = -1, j_min = -1;
        for(int i = 0; i < n; i++){
            if ((I[i] == 0) || PQ[i].empty())
                continue;
            pair<long long int, int> min_elem = PQ[i].begin()->first;
            if (min_elem.first < curr_min){
                curr_min = min_elem.first;
                i_min = i;
                j_min = min_elem.second;
            }
        }
        if (i_min == -1 || j_min == -1)
            break;
        dsu.merge(i_min, j_min);
        int set_i = dsu.root(i_min);
        if (set_i != i_min)
            throw "PROBLEM IN HAC \n";
        // order_graph[set_i][set_i] = get_routeplan_cost(order_graph[i_min][j_min].first.first, global_time);
        double new_cost = order_graph[i_min][j_min].first.second + order_graph[i_min][i_min].first.second + order_graph[j_min][j_min].first.second;
        order_graph[set_i][set_i] = order_graph[i_min][j_min];
        order_graph[set_i][set_i].first.second = new_cost;
        total_sum += order_graph[i_min][j_min].first.second;
        total_batches--;
        I[j_min] = 0;
        PQ[i_min].clear();
        for(int i = 0; i < j_min; i++){
            if (I[i] == 1){
                double cost = order_graph[i][j_min].first.second;
                PQ[i].erase({(long long int)(cost*double_to_int), j_min});
            }
        }
        for(int i = 0; i < i_min; i++){
            if (I[i] == 1){
                double cost = order_graph[i][i_min].first.second;
                PQ[i].erase({(long long int)(cost*double_to_int), i_min});
            }
        }
        int size_set_i = dsu.set_to_elems[set_i].size();
        for(int i = 0; i < i_min; i++){
            if (I[i] == 1){
                int size_i = dsu.get_elems(i).size();
                if (size_set_i + size_i > batching_cap ||
                    (order_graph[i][i_min].first.second + FP_EPSILON >= MAX_NUM) ||
                    (order_graph[i][j_min].first.second + FP_EPSILON >= MAX_NUM)){
                        continue;
                }
                order_graph[i][set_i] = get_inserted_bestplan(order_graph[set_i][set_i].first.first,
                                                              order_graph[i][i].first.first, global_time);
                order_graph[i][set_i].first.second -= (order_graph[set_i][set_i].first.second + order_graph[i][i].first.second);
                double cost = order_graph[i][set_i].first.second;
                if (cost + FP_EPSILON >= MAX_NUM)
                    continue;
                PQ[i][{(long long int)(cost*double_to_int), set_i}] = {i,set_i};
            }
        }
        for(int i = set_i+1; i < n; i++){
            if (I[i] == 1){
                int size_i = dsu.get_elems(i).size();
                if (size_set_i + size_i > batching_cap ||
                    (order_graph[i_min][i].first.second + FP_EPSILON >= MAX_NUM) ||
                    (order_graph[i][j_min].first.second + FP_EPSILON >= MAX_NUM && order_graph[j_min][i].first.second + FP_EPSILON >= MAX_NUM)){
                        continue;
                }
                order_graph[set_i][i] = get_inserted_bestplan(order_graph[set_i][set_i].first.first,
                                                              order_graph[i][i].first.first, global_time);
                order_graph[set_i][i].first.second -= (order_graph[set_i][set_i].first.second + order_graph[i][i].first.second);
                double cost = order_graph[set_i][i].first.second;
                if (cost + FP_EPSILON >= MAX_NUM)
                    continue;
                PQ[set_i][{(long long int)(cost*double_to_int), i}] = {set_i,i};
            }
        }
        iter_c++;
    }
    vector<best_plan_tuple> cluster_pack;
    for(auto it:dsu.set_to_elems){
        cluster_pack.push_back(order_graph[it.first][it.first]);
    }
    return cluster_pack;
}



// Given two routeplans, returns a list of interleaving routeplans
// while keeping the sequence of input routeplans fixed
vector<vector<event>> gen_inserted_sequence_rp(vector<event> &rp0, vector<event> &rp1){
    vector<vector<event>> ans;
    int n0 = rp0.size();
    int n1 = rp1.size();
    vector<int> perm(n0 + n1, 0);
    for(int i = n0; i < n0+n1; i++){
        perm[i] = 1;
    }
    do {
        vector<event> new_rp(n0 + n1);
        int cnt0=0;
        int cnt1=0;
        for(auto it:perm){
            if (it == 0){
                new_rp[cnt1+cnt0] = rp0[cnt0];
                cnt0++;
            }
            else{
                new_rp[cnt1+cnt0] = rp1[cnt1];
                cnt1++;
            }
        }
        ans.push_back(new_rp);
    } while (next_permutation(perm.begin(),perm.end()));
    return ans;
}

void foodmatch_BR(vector<int> &active_vehicles, vector<order> &active_orders,
                  double global_time, vector<order> &rejected_orders){
    auto start = std::chrono::high_resolution_clock::now();
    // Reshuffling
    if (global_time <= end_time + FP_EPSILON){
        for(auto idx:active_vehicles){
            vehicle* vh = &all_vehicles[idx];
            if ((vh->route_plan).empty())
                continue;
            vector<event> new_route_plan;
            vector<order> new_order_set;
            unordered_map<string, int> picked_ids;
            bool latest_pickup_event = ((vh->route_plan[0]).type == 0);
            for (auto evt:(vh->route_plan)){
                if (evt.type == 0){
                    active_orders.push_back(evt.order_obj);
                    picked_ids[evt.order_obj.order_id] = 1;
                }
                else{
                    if (picked_ids.find(evt.order_obj.order_id) == picked_ids.end()){
                        new_route_plan.push_back(evt);
                        new_order_set.push_back(evt.order_obj);
                    }
                }
            }
            if(new_order_set.empty()){
                (vh->order_set) = {};
                (vh->route_plan) = {};
            }
            else{
                if (latest_pickup_event){
                    (vh->order_set).clear();
                    vh->assign_order_pack(new_order_set, new_route_plan, global_time);
                }
                else{
                    (vh->order_set) = new_order_set;
                    (vh->route_plan) = new_route_plan;
                }
            }
        }
    }
    auto stop3 = std::chrono::high_resolution_clock::now();
    auto duration3 = std::chrono::duration_cast<std::chrono::microseconds>(stop3 - start);
    if (VERBOSITY == -1)
        cout << "collect_time," << duration3.count() << endl;
    vector<best_plan_tuple> bp_packs = hac_cluster_orders(active_orders, global_time);
    vector<vector<order>> order_packs(bp_packs.size());
    for(int i = 0; i < int(bp_packs.size()); i++){
        for (auto ev:bp_packs[i].first.first){
            if (ev.type == 0)
                order_packs[i].push_back(ev.order_obj);
        }
    }
    auto stop2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - start);
    if (VERBOSITY == -1)
        cout << "cluster_time," << duration2.count() << endl;
    vector<vector<double>> cost_mat(active_vehicles.size(), vector<double>(order_packs.size(), MAX_NUM));
    vector<vector<best_plan_tuple>> best_plans(active_vehicles.size(), vector<best_plan_tuple>(order_packs.size()));
    for(int i = 0; i < int(active_vehicles.size()); i++){
        vehicle vh = all_vehicles[active_vehicles[i]];
        vector<event> org_route_plan = vh.route_plan;
        unordered_map<string, double> org_d_times = get_delivered_times(vh, org_route_plan, global_time);
        double org_cost = get_route_plan_extra_delivery_time(org_d_times, org_route_plan);
        for(int j = 0; j < int(order_packs.size()); j++){
            double min_cost = MAX_NUM;
            vector<event> min_plan;
            unordered_map<string, double> min_delivery_times;
            vector<order> op = order_packs[j];
            vector<event> bp = bp_packs[j].first.first;
            double vh_to_rest_dist = MAX_NUM;
            if (bp.size() > 0)
                vh_to_rest_dist = hhl_sp_query(vh.get_current_location(), bp[0].node);
            if ((vh_to_rest_dist >= vehicle_rest_radius_cap) || (int(int(vh.order_set.size() + op.size())) > batching_cap))
                continue;
            vector<vector<event>> all_route_plans = gen_inserted_sequence_rp(org_route_plan, bp);
            for(int ri = 0; ri < int(all_route_plans.size()); ri++){
                bool cap_compatible = check_capacity_constraint(vh, all_route_plans[ri]);
                unordered_map<string, double> d_times = get_delivered_times(vh, all_route_plans[ri], global_time);
                if (d_times.empty() || !cap_compatible)
                    continue;
                double plan_cost = get_route_plan_extra_delivery_time(d_times, all_route_plans[ri]) - org_cost;
                if (plan_cost < min_cost){
                    min_cost = plan_cost;
                    min_plan = all_route_plans[ri];
                    min_delivery_times = d_times;
                }
            }
            cost_mat[i][j] = min_cost;
            best_plans[i][j] = {{min_plan, min_cost}, min_delivery_times};
        }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if (VERBOSITY == -1)
        cout << "cost_time," << duration.count() << endl;
    start = std::chrono::high_resolution_clock::now();
    double min_cost = 0;
    for(auto it:cost_mat){
        for(auto it2:it)
            min_cost = min(min_cost, it2);
    }
    // Make cost matrix positive
    // Shortest delivery time is calculated based on graph of one timeslot
    // During simulation, timeslot and therefore graphs may change resulting in negative costs
    for(int i = 0; i < int(active_vehicles.size()); i++){
        for(int j = 0; j < int(order_packs.size()); j++){
            cost_mat[i][j] = (cost_mat[i][j] > max_cost_val)?max_cost_val:cost_mat[i][j];
            cost_mat[i][j] = cost_mat[i][j] + max(0.0, -min_cost);
        }
    }
    // order index assigned to each vehicle (-1 if not assigned an order)
    vector<int> assignment(cost_mat.size(), -1);
    if (int(cost_mat.size()) > 0 && int(cost_mat[0].size()) > 0){
        double cost = HUN_ASSIGN(cost_mat, assignment);
    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if (VERBOSITY == -1)
        cout << "hungarian_time," << duration.count() << endl;
    for(int i = 0; i < int(assignment.size()); i++){
        if(assignment[i] != -1){
            if (cost_mat[i][assignment[i]] + FP_EPSILON >= (max_cost_val + max(0.0, -min_cost)))
                assignment[i] = -1;
        }
    }
    // Perform respective assignments
    unordered_map<int, int> vehicle_assigned;
    for(int i = 0; i < int(assignment.size()); i++){
        if (assignment[i] != -1){
            all_vehicles[active_vehicles[i]].assign_order_pack(order_packs[assignment[i]],
                                                            best_plans[i][assignment[i]].first.first, global_time);
            if (VERBOSITY == -1){
                for (auto ord_obj:order_packs[assignment[i]])
                    cout << fixed << "ASSIGN,"<< ord_obj.order_id << "," << all_vehicles[active_vehicles[i]].vehicle_id << ","<< global_time << endl;
            }
            vehicle_assigned[assignment[i]] = i;
        }
    }
    // fill rejected orders in this round
    for (int i = 0; i < int(order_packs.size()); i++){
        if (vehicle_assigned.find(i)==vehicle_assigned.end()){
            for (int j = 0; j < int(order_packs[i].size()); j++){
                rejected_orders.push_back(order_packs[i][j]);
            }
        }
    }
}

void foodmatch_BFS(vector<int> &active_vehicles, vector<order> &active_orders,
                   double global_time, vector<order> &rejected_orders){

    auto start = std::chrono::high_resolution_clock::now();
    int curr_time_slot = ((((long long int)(global_time))%86400 + 86400)%86400)/3600;
    if (global_time <= end_time + FP_EPSILON){
        for(auto idx:active_vehicles){
            vehicle* vh = &all_vehicles[idx];
            if ((vh->route_plan).empty())
                continue;
            vector<event> new_route_plan;
            vector<order> new_order_set;
            unordered_map<string, int> picked_ids;
            bool latest_pickup_event = ((vh->route_plan[0]).type == 0);
            for (auto evt:(vh->route_plan)){
                if (evt.type == 0){
                    active_orders.push_back(evt.order_obj);
                    picked_ids[evt.order_obj.order_id] = 1;
                }
                else{
                    if (picked_ids.find(evt.order_obj.order_id) == picked_ids.end()){
                        new_route_plan.push_back(evt);
                        new_order_set.push_back(evt.order_obj);
                    }
                }
            }
            if(new_order_set.empty()){
                (vh->order_set) = {};
                (vh->route_plan) = {};
            }
            else{
                if (latest_pickup_event){
                    (vh->order_set).clear();
                    vh->assign_order_pack(new_order_set, new_route_plan, global_time);
                }
                else{
                    (vh->order_set) = new_order_set;
                    (vh->route_plan) = new_route_plan;
                }
            }
        }
    }
    auto stop3 = std::chrono::high_resolution_clock::now();
    auto duration3 = std::chrono::duration_cast<std::chrono::microseconds>(stop3 - start);
    if (VERBOSITY == -1)
        cout << "collect_time," << duration3.count() << endl;
    vector<best_plan_tuple> bp_packs = hac_cluster_orders_vh(active_orders, global_time);
    vector<vector<order>> order_packs(bp_packs.size());
    for(int i = 0; i < int(bp_packs.size()); i++){
        for (auto ev:bp_packs[i].first.first){
            if (ev.type == 0)
                order_packs[i].push_back(ev.order_obj);
        }
    }
    auto stop2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - start);
    if (VERBOSITY == -1)
        cout << "cluster_time," << duration2.count() << endl;
    vector<vector<double>> cost_mat(active_vehicles.size(), vector<double>(order_packs.size(), MAX_NUM));
    vector<vector<best_plan_tuple>> best_plans(active_vehicles.size(), vector<best_plan_tuple>(order_packs.size()));
    vector<vector<int>> node_to_order_pack(nodes_to_latlon.size());
    for (int i = 0; i < int(bp_packs.size()); i++)
        node_to_order_pack[bp_packs[i].first.first[0].node].push_back(i);
    int max_count_edges = int(double(active_orders.size())/double(active_vehicles.size()) * double(vehicle_explore_frac));
    max_count_edges = max(1, max_count_edges);
    double max_time_dist = vehicle_explore_rad;
    vector<bool> visited(nodes_to_latlon.size(), false);
    vector<double> dist_vh(nodes_to_latlon.size(), MAX_NUM);
    for(int vhi = 0; vhi < int(active_vehicles.size()); vhi++){
        if (bp_packs.size() == 0)
            continue;
        vehicle vh = all_vehicles[active_vehicles[vhi]];
        vector<event> org_route_plan = vh.route_plan;
        unordered_map<string, double> org_d_times = get_delivered_times(vh, org_route_plan, global_time);
        double org_cost = get_route_plan_extra_delivery_time(org_d_times, org_route_plan);
        int count_edges = 0;
        fill(visited.begin(), visited.end(), false);
        fill(dist_vh.begin(), dist_vh.end(), MAX_NUM);
        long long int vh_start_node = vh.get_current_location();
        dist_vh[vh_start_node] = 0.0;
        priority_queue<pair<double, long long int>> pq;
        // dist + h()
        pq.push({-dist_vh[vh_start_node], vh_start_node});
        double dist, alt;
        long long int u, v;
        while(!pq.empty()){
            pair<double, long long int> top = pq.top();
            pq.pop();
            u = top.second;
            if (visited[u])
                continue;
            if (count_edges > max_count_edges || dist_vh[u] > 1.0)
                break;
            visited[u] = true;
            if (node_to_order_pack[u].size() != 0){
                for(auto pack_idx : node_to_order_pack[u]){
                    if (count_edges > max_count_edges)
                        break;
                    double min_cost = MAX_NUM;
                    vector<event> min_plan;
                    unordered_map<string, double> min_delivery_times;
                    vector<order> op = order_packs[pack_idx];
                    vector<event> bp = bp_packs[pack_idx].first.first;
                    if ((int(vh.order_set.size() + op.size()) > batching_cap))
                        continue;
                    vector<vector<event>> all_route_plans = gen_inserted_sequence_rp(org_route_plan, bp);
                    for(int ri = 0; ri < int(all_route_plans.size()); ri++){
                        bool cap_compatible = check_capacity_constraint(vh, all_route_plans[ri]);
                        unordered_map<string, double> d_times = get_delivered_times(vh, all_route_plans[ri], global_time);
                        if (d_times.empty() || !cap_compatible)
                            continue;
                        double plan_cost = get_route_plan_extra_delivery_time(d_times, all_route_plans[ri]) - org_cost;
                        if (plan_cost < min_cost){
                            min_cost = plan_cost;
                            min_plan = all_route_plans[ri];
                            min_delivery_times = d_times;
                        }
                    }
                    cost_mat[vhi][pack_idx] = min_cost;
                    best_plans[vhi][pack_idx] = {{min_plan, min_cost}, min_delivery_times};
                    if (min_cost + FP_EPSILON < MAX_NUM)
                        count_edges++;
                }
            }
            if (count_edges > max_count_edges)
                break;
            for(int ei = 0; ei < int(edges[u].size()); ei++){
                v = edges[u][ei];
                double e_weight = get_edge_weight(u, v, curr_time_slot);
                alt = dist_vh[u] + e_weight/max_time_dist;
                if (alt < dist_vh[v]){
                    dist_vh[v] = alt;
                    pq.push(make_pair(-alt, v));
                }
            }
        }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if (VERBOSITY == -1)
        cout << "cost_time," << duration.count() << endl;
    start = std::chrono::high_resolution_clock::now();
    double min_cost = 0;
    for(auto it:cost_mat){
        for(auto it2:it)
            min_cost = min(min_cost, it2);
    }
    // Make cost matrix positive
    // Shortest delivery time is calculated based on graph of one timeslot
    // During simulation, timeslot and therefore graphs may change resulting in negative costs
    for(int i = 0; i < int(active_vehicles.size()); i++){
        for(int j = 0; j < int(order_packs.size()); j++){
            cost_mat[i][j] = (cost_mat[i][j] > max_cost_val)?max_cost_val:cost_mat[i][j];
            cost_mat[i][j] = cost_mat[i][j] + max(0.0, -min_cost);
        }
    }
    // order index assigned to each vehicle (-1 if not assigned an order)
    vector<int> assignment(cost_mat.size(), -1);
    if (int(cost_mat.size()) > 0 && int(cost_mat[0].size()) > 0){
        double cost = HUN_ASSIGN(cost_mat, assignment);
    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if (VERBOSITY == -1)
        cout << "hungarian_time," << duration.count() << endl;
    for(int i = 0; i < int(assignment.size()); i++){
        if(assignment[i] != -1){
            if (cost_mat[i][assignment[i]] + FP_EPSILON >= (max_cost_val + max(0.0, -min_cost)))
                assignment[i] = -1;
        }
    }
    // Perform respective assignments
    unordered_map<int, int> vehicle_assigned;
    for(int i = 0; i < int(assignment.size()); i++){
        if (assignment[i] != -1){
            all_vehicles[active_vehicles[i]].assign_order_pack(order_packs[assignment[i]],
                                                            best_plans[i][assignment[i]].first.first, global_time);
            if (VERBOSITY > 0){
                for (auto ord_obj:order_packs[assignment[i]])
                    cout << "Vehicle :" << all_vehicles[active_vehicles[i]].vehicle_id << " | Order : " <<  ord_obj.order_id << endl;
            }
            if (VERBOSITY == -1){
                for (auto ord_obj:order_packs[assignment[i]])
                    cout << fixed << "ASSIGN,"<< ord_obj.order_id << "," << all_vehicles[active_vehicles[i]].vehicle_id << ","<< global_time << endl;
            }
            vehicle_assigned[assignment[i]] = i;
        }
    }
    // fill rejected orders in this round
    for (int i = 0; i < int(order_packs.size()); i++){
        if (vehicle_assigned.find(i)==vehicle_assigned.end()){
            for (int j = 0; j < int(order_packs[i].size()); j++){
                rejected_orders.push_back(order_packs[i][j]);
            }
        }
    }
}

void foodmatch_FULL(vector<int> &active_vehicles, vector<order> &active_orders,
                    double global_time, vector<order> &rejected_orders){
    auto start = std::chrono::high_resolution_clock::now();
    int curr_time_slot = ((((long long int)(global_time))%86400 + 86400)%86400)/3600;
    if (global_time <= end_time + FP_EPSILON){
        for(auto idx:active_vehicles){
            vehicle* vh = &all_vehicles[idx];
            if ((vh->route_plan).empty())
                continue;
            vector<event> new_route_plan;
            vector<order> new_order_set;
            unordered_map<string, int> picked_ids;
            bool latest_pickup_event = ((vh->route_plan[0]).type == 0);
            for (auto evt:(vh->route_plan)){
                if (evt.type == 0){
                    active_orders.push_back(evt.order_obj);
                    picked_ids[evt.order_obj.order_id] = 1;
                }
                else{
                    if (picked_ids.find(evt.order_obj.order_id) == picked_ids.end()){
                        new_route_plan.push_back(evt);
                        new_order_set.push_back(evt.order_obj);
                    }
                }
            }
            if(new_order_set.empty()){
                (vh->order_set) = {};
                (vh->route_plan) = {};
            }
            else{
                if (latest_pickup_event){
                    (vh->order_set).clear();
                    vh->assign_order_pack(new_order_set, new_route_plan, global_time);
                }
                else{
                    (vh->order_set) = new_order_set;
                    (vh->route_plan) = new_route_plan;
                }
            }
        }
    }
    auto stop3 = std::chrono::high_resolution_clock::now();
    auto duration3 = std::chrono::duration_cast<std::chrono::microseconds>(stop3 - start);
    if (VERBOSITY == -1)
        cout << "collect_time," << duration3.count() << endl;
    vector<best_plan_tuple> bp_packs = hac_cluster_orders_as(active_orders, global_time);
    vector<vector<order>> order_packs(bp_packs.size());
    for(int i = 0; i < int(bp_packs.size()); i++){
        for (auto ev:bp_packs[i].first.first){
            if (ev.type == 0)
                order_packs[i].push_back(ev.order_obj);
        }
    }
    auto stop2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - start);
    if (VERBOSITY == -1)
        cout << "cluster_time," << duration2.count() << endl;
    vector<vector<double>> cost_mat(active_vehicles.size(), vector<double>(order_packs.size(), MAX_NUM));
    vector<vector<best_plan_tuple>> best_plans(active_vehicles.size(), vector<best_plan_tuple>(order_packs.size()));
    vector<vector<int>> node_to_order_pack(nodes_to_latlon.size());
    for (int i = 0; i < int(bp_packs.size()); i++)
        node_to_order_pack[bp_packs[i].first.first[0].node].push_back(i);
    int max_count_edges = int(double(active_orders.size())/double(active_vehicles.size()) * double(vehicle_explore_frac));
    max_count_edges = max(1, max_count_edges);
    double max_time_dist = slotted_max_time[curr_time_slot];
    vector<int> pack_deg(bp_packs.size(), 0);
    vector<bool> visited(nodes_to_latlon.size(), false);
    vector<double> dist_vh(nodes_to_latlon.size(), MAX_NUM);
    vector<double> dist_ac(nodes_to_latlon.size(), MAX_NUM);
    if ((global_time <= end_time + FP_EPSILON) && (active_orders.size() > 0)){
        for(int vhi = 0; vhi < int(active_vehicles.size()); vhi++){
            if (bp_packs.size() == 0)
                continue;
            vehicle vh = all_vehicles[active_vehicles[vhi]];
            vector<event> org_route_plan = vh.route_plan;
            unordered_map<string, double> org_d_times = get_delivered_times(vh, org_route_plan, global_time);
            double org_cost = get_route_plan_extra_delivery_time(org_d_times, org_route_plan);
            int count_edges = 0;
            fill(visited.begin(), visited.end(), false);
            fill(dist_vh.begin(), dist_vh.end(), MAX_NUM);
            fill(dist_ac.begin(), dist_ac.end(), MAX_NUM);
            long long int vh_start_node = vh.get_current_location();
            dist_vh[vh_start_node] = 0.0;
            dist_ac[vh_start_node] = 0.0;
            priority_queue<pair<double, long long int>> pq;
            // dist + h()
            pq.push({-dist_vh[vh_start_node], vh_start_node});
            double dist, alt;
            long long int u, v;
            while(!pq.empty()){
                pair<double, long long int> top = pq.top();
                pq.pop();
                u = top.second;
                if (visited[u])
                    continue;
                if (count_edges > max_count_edges)
                    break;
                if (dist_ac[u] > vehicle_rest_radius_cap){
                    if (hhl_sp_query(vh_start_node, u) > vehicle_rest_radius_cap)
                        break;
                }
                visited[u] = true;
                if (node_to_order_pack[u].size() != 0){
                    for(auto pack_idx : node_to_order_pack[u]){
                        if (count_edges > max_count_edges)
                            break;
                        double min_cost = MAX_NUM;
                        vector<event> min_plan;
                        unordered_map<string, double> min_delivery_times;
                        vector<order> op = order_packs[pack_idx];
                        vector<event> bp = bp_packs[pack_idx].first.first;
                        if ((int(vh.order_set.size() + op.size()) > batching_cap))
                            continue;
                        vector<vector<event>> all_route_plans = gen_inserted_sequence_rp(org_route_plan, bp);
                        for(int ri = 0; ri < int(all_route_plans.size()); ri++){
                            bool cap_compatible = check_capacity_constraint(vh, all_route_plans[ri]);
                            unordered_map<string, double> d_times = get_delivered_times(vh, all_route_plans[ri], global_time);
                            if (d_times.empty() || !cap_compatible)
                                continue;
                            double plan_cost = get_route_plan_extra_delivery_time(d_times, all_route_plans[ri]) - org_cost;
                            if (plan_cost < min_cost){
                                min_cost = plan_cost;
                                min_plan = all_route_plans[ri];
                                min_delivery_times = d_times;
                            }
                        }
                        cost_mat[vhi][pack_idx] = min_cost;
                        best_plans[vhi][pack_idx] = {{min_plan, min_cost}, min_delivery_times};
                        if (min_cost + FP_EPSILON < MAX_NUM)
                            count_edges++;
                    }
                }
                if (count_edges > max_count_edges)
                    break;
                for(int ei = 0; ei < int(edges[u].size()); ei++){
                    v = edges[u][ei];
                    double e_weight = get_edge_weight(u, v, curr_time_slot);
                    double heur_val = heuristic_function(vh, v);
                    alt = dist_vh[u] + (1-heuristic_multiplier)*(e_weight)/max_time_dist + heuristic_multiplier*heur_val;
                    if (alt < dist_vh[v]){
                        dist_ac[v] = dist_ac[u] + e_weight;
                        dist_vh[v] = alt;
                        pq.push(make_pair(-alt, v));
                    }
                }
            }
        }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if (VERBOSITY == -1)
        cout << "cost_time," << duration.count() << endl;
    start = std::chrono::high_resolution_clock::now();
    double min_cost = 0;
    for(auto it:cost_mat){
        for(auto it2:it)
            min_cost = min(min_cost, it2);
    }
    // Make cost matrix positive
    // Shortest delivery time is calculated based on graph of one timeslot
    // During simulation, timeslot and therefore graphs may change resulting in negative costs
    for(int i = 0; i < int(active_vehicles.size()); i++){
        for(int j = 0; j < int(order_packs.size()); j++){
            cost_mat[i][j] = (cost_mat[i][j] > max_cost_val)?max_cost_val:cost_mat[i][j];
            cost_mat[i][j] = cost_mat[i][j] + max(0.0, -min_cost);
        }
    }
    // order index assigned to each vehicle (-1 if not assigned an order)
    vector<int> assignment(cost_mat.size(), -1);
    if (int(cost_mat.size()) > 0 && int(cost_mat[0].size()) > 0){
        double cost = HUN_ASSIGN(cost_mat, assignment);
    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if (VERBOSITY == -1)
        cout << "hungarian_time," << duration.count() << endl;
    for(int i = 0; i < int(assignment.size()); i++){
        if(assignment[i] != -1){
            if (cost_mat[i][assignment[i]] + FP_EPSILON >= (max_cost_val + max(0.0, -min_cost)))
                assignment[i] = -1;
        }
    }
    // Perform respective assignments
    unordered_map<int, int> vehicle_assigned;
    for(int i = 0; i < int(assignment.size()); i++){
        if (assignment[i] != -1){
            all_vehicles[active_vehicles[i]].assign_order_pack(order_packs[assignment[i]],
                                                            best_plans[i][assignment[i]].first.first, global_time);
            if (VERBOSITY == -1){
                for (auto ord_obj:order_packs[assignment[i]]){
                    
                    //cout << fixed << "ASSIGN,"<< ord_obj.order_id << "," << all_vehicles[active_vehicles[i]].vehicle_id << ","<< global_time << endl;
                    vehicle ass_vh = all_vehicles[active_vehicles[i]];
					 cout << fixed << "ASSIGN,"<< ord_obj.order_id << "," << ass_vh.vehicle_id << ","<< global_time << ",";
					 cout<< nodes_to_latlon[ass_vh.path[ass_vh.path_present_idx]].first << "," << nodes_to_latlon[ass_vh.path[ass_vh.path_present_idx]].second << ",";
					 cout<< ord_obj.restaurant.rest_latlon << "," << ord_obj.customer.cust_latlon << endl;
                }
            }
            vehicle_assigned[assignment[i]] = i;
        }
    }
    // fill rejected orders in this round
    for (int i = 0; i < int(order_packs.size()); i++){
        if (vehicle_assigned.find(i)==vehicle_assigned.end()){
            for (int j = 0; j < int(order_packs[i].size()); j++){
                rejected_orders.push_back(order_packs[i][j]);
            }
        }
    }
}


//Fairness algo with BestFS from each order to find vehicle within SLA from order
void fair_foody(vector<int> &active_vehicles, vector<order> &active_orders,
													  double global_time, vector<order> &rejected_orders){
	//payment cofficients
	double lambda_1 =  0.8; //rupees paid per unit time of wait
	double lambda_2 =  1 ;  //rupees paid per unit time of travel.
	double max_pay_val = 100000;
	auto start = std::chrono::high_resolution_clock::now();

	//int curr_time_slot = ((((long long int)(global_time - global_conf.DAY_TIMESTAMP))%86400 + 86400)%86400)/3600;
    int curr_time_slot = ((((long long int)(global_time))%86400 + 86400)%86400)/3600;

	//cout << global_time << " " << end_time + FP_EPSILON << endl;

	cout << "UNPICK ORDERS = " << active_orders.size() << endl;

	auto stop3 = std::chrono::high_resolution_clock::now();
	auto duration3 = std::chrono::duration_cast<std::chrono::microseconds>(stop3 - start);
	if (VERBOSITY == -1)
		cout << "collect_time," << duration3.count() << endl;

	vector<best_plan_tuple> bp_packs = hac_cluster_orders_as_restrict(active_orders, global_time, active_vehicles.size());
	
	vector<vector<order>> order_packs(bp_packs.size());
	for(int i = 0; i < int(bp_packs.size()); i++){
		for (auto ev:bp_packs[i].first.first){
			if (ev.type == 0)
				order_packs[i].push_back(ev.order_obj);
		}
	}
	
	auto stop2 = std::chrono::high_resolution_clock::now();
	auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - start);
	if (VERBOSITY == -1)
		cout << "cluster_time," << duration2.count() << endl;

	vector<vector<double>> cost_mat(active_vehicles.size(), vector<double>(order_packs.size(), MAX_NUM));
	vector<vector<best_plan_tuple>> best_plans(active_vehicles.size(), vector<best_plan_tuple>(order_packs.size()));

	vector<vector<int>> node_to_order_pack(nodes_to_latlon.size());
	for (int i = 0; i < int(bp_packs.size()); i++)
		node_to_order_pack[bp_packs[i].first.first[0].node].push_back(i);
	
	vector<vector<int>> node_to_vehicle(nodes_to_latlon.size());
	for (int i = 0; i < int(active_vehicles.size()); i++)
		node_to_vehicle[all_vehicles[active_vehicles[i]].get_current_location()].push_back(i);
	
	int max_count_edges = int(double(active_orders.size())/double(active_vehicles.size()) * double(vehicle_explore_frac));
	max_count_edges = max(1, max_count_edges);

	double max_time_dist = slotted_max_time[curr_time_slot];
	
	double dj_time = 0.0;

	vector<int> pack_deg(bp_packs.size(), 0);

	vector<bool> visited(nodes_to_latlon.size(), false);
	vector<double> dist_vh(nodes_to_latlon.size(), MAX_NUM);
	vector<double> dist_ac(nodes_to_latlon.size(), MAX_NUM);

	//cout << global_time << " MJ " << end_time + FP_EPSILON << endl;
	if ((global_time <= end_time + FP_EPSILON) && (active_orders.size() > 0)){
		cout << "AS IN AS" << endl;
		double min_payment = MAX_NUM;
		//vehicle vh_min_pay;
		//vector<best_plan_tuple> best_plans_vh(vector<best_plan_tuple>(order_packs.size()));
		for(int vhi = 0; vhi < int(active_vehicles.size()); vhi++){
			vehicle vh = all_vehicles[active_vehicles[vhi]];
			double vh_payment = (lambda_1 * vh.wait_time + lambda_2 * vh.travel_time)/(60*(vh.wait_time+vh.travel_time+vh.idle_time));
			if(vh_payment < min_payment){
				min_payment = vh_payment;
				//vh_min_pay = vh;
			}
		}
		//for(int vhi = 0; vhi < int(active_vehicles.size()); vhi++){
		for(int pack_idx = 0; pack_idx < int(bp_packs.size()); pack_idx++){
			if (bp_packs.size() == 0)
				continue;
			vector<order> op = order_packs[pack_idx];
			vector<event> bp = bp_packs[pack_idx].first.first;

			int count_edges = 0;

			fill(visited.begin(), visited.end(), false);
			fill(dist_vh.begin(), dist_vh.end(), MAX_NUM);
			fill(dist_ac.begin(), dist_ac.end(), MAX_NUM);
			long long int vh_start_node = bp_packs[pack_idx].first.first[0].node;
			
			dist_vh[vh_start_node] = 0.0;
			dist_ac[vh_start_node] = 0.0;
			
			priority_queue<pair<double, long long int>> pq;

			pq.push({-dist_ac[vh_start_node], vh_start_node});

			double dist, alt;
			long long int u, v;
            //vehicle nearest_vh;
            double nearest_vh_distance = -1;
            bool nearest_vh_found = false;
            bool distance_exceeded = false;

			auto dj_start = std::chrono::high_resolution_clock::now();
			while(!pq.empty()){
				pair<double, long long int> top = pq.top();
				pq.pop();

				u = top.second;

				if (visited[u])
					continue;
				
				//if (count_edges > max_count_edges)
				//	break;
				if (distance_exceeded == true)
					break;

				if (dist_ac[u] > vehicle_rest_radius_cap){
					if (hhl_sp_query(vh_start_node, u) > vehicle_rest_radius_cap)
						break;
				}

				visited[u] = true;
				if (node_to_vehicle[u].size() != 0){
					// if (hhl_sp_query(vh_start_node, u) < vehicle_rest_radius_cap){
						
						for(auto vhi : node_to_vehicle[u]){
							if (nearest_vh_found == true and dist_ac[u]>nearest_vh_distance*(100+pct_explore_frac)/100){
								distance_exceeded = true;
								break;
							}
							vehicle vh = all_vehicles[active_vehicles[vhi]];
							if (nearest_vh_found == false){                               
								nearest_vh_distance = dist_ac[u];
								//nearest_vh_found = true;
                                //cout<<"found nearest vehicle at distance "<<nearest_vh_distance<<endl;
							}
                            
							double min_cost = MAX_NUM;
                            vector<event> org_route_plan = vh.route_plan;
							unordered_map<string, double> org_d_times = get_delivered_times(vh, org_route_plan, global_time);
							double org_cost;
// 							if (cost_type=="EDT")
// 								org_cost = get_route_plan_extra_delivery_time(org_d_times, org_route_plan);
// 							else if (cost_type=="NEDT")
// 								org_cost = get_route_plan_normalized_extra_delivery_time(org_d_times, org_route_plan);
// 							else{
// 								throw invalid_argument("COST TYPE NOT RECOGNISED\n");
// 							}
                            org_cost = get_route_plan_extra_delivery_time(org_d_times, org_route_plan);
							vector<event> min_plan;
							unordered_map<string, double> min_delivery_times;
							if ((int(vh.order_set.size() + op.size()) > batching_cap))
								continue;
							
							vector<vector<event>> all_route_plans = gen_inserted_sequence_rp(org_route_plan, bp);
							for(int ri = 0; ri < int(all_route_plans.size()); ri++){
								bool cap_compatible = check_capacity_constraint(vh, all_route_plans[ri]);

								unordered_map<string, double> d_times = get_delivered_times(vh, all_route_plans[ri], global_time);
								
								if (d_times.empty() || !cap_compatible)
									continue;

								bool sla_compatible = check_route_plan_sla_constraint(d_times, all_route_plans[ri]);

								//if (!sla_compatible)
									//continue;

								double plan_cost;
// 								if (cost_type=="EDT")
// 									plan_cost = get_route_plan_extra_delivery_time(d_times, all_route_plans[ri]) - org_cost;
// 								else if (cost_type=="NEDT")
// 									plan_cost = get_route_plan_normalized_extra_delivery_time(d_times, all_route_plans[ri]) - org_cost;
// 								else{
// 									throw invalid_argument("COST TYPE NOT RECOGNISED\n");
// 								}
								plan_cost = get_route_plan_extra_delivery_time(d_times, all_route_plans[ri]) - org_cost;
                                
								if (plan_cost < min_cost){
									min_cost = plan_cost;
									min_plan = all_route_plans[ri];
									min_delivery_times = d_times;
								}
							}
							double total_delivery_time = get_route_plan_total_delivery_time(min_delivery_times, min_plan, global_time);
							double vh_payment = (lambda_1 * vh.wait_time + lambda_2 * vh.travel_time+ lambda_2*total_delivery_time)/(60 * (vh.wait_time+vh.travel_time+vh.idle_time+total_delivery_time));
							double vh_payment_gap = vh_payment - min_payment;
							//cost_mat[vhi][pack_idx] = min_cost;
							cost_mat[vhi][pack_idx] = vh_payment_gap;
							best_plans[vhi][pack_idx] = {{min_plan, min_cost}, min_delivery_times};
							if (min_cost + FP_EPSILON < MAX_NUM)
								count_edges++;

						}
					//}
				}
				if (distance_exceeded == true)
					break;
                    
                

				for(int ei = 0; ei < int(edges[u].size()); ei++){
					v = edges[u][ei];
					double e_weight = get_edge_weight(u, v, curr_time_slot);
					dist_ac[v] = dist_ac[u] + e_weight;
					pq.push(make_pair(-dist_ac[v], v));                    
				}
			}
			auto dj_stop = std::chrono::high_resolution_clock::now();
			auto dj_duration = std::chrono::duration_cast<std::chrono::microseconds>(dj_stop - dj_start); 

			dj_time += dj_duration.count();
			
		}
	}
	cout << "dj_time," << dj_time << endl;

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
	if (VERBOSITY == -1)
		cout << "cost_time," << duration.count() << endl;

	start = std::chrono::high_resolution_clock::now();
	
	double min_cost = 0;
	for(auto it:cost_mat){
		for(auto it2:it)
			min_cost = min(min_cost, it2);
	}

	// Make cost matrix positive
	// Shortest delivery time is calculated based on graph of one timeslot
	// During simulation, timeslot and therefore graphs may change resulting in negative costs
	for(int i = 0; i < int(active_vehicles.size()); i++){
		for(int j = 0; j < int(order_packs.size()); j++){
            cost_mat[i][j] = (cost_mat[i][j] > max_pay_val)?max_pay_val:cost_mat[i][j];
			cost_mat[i][j] = cost_mat[i][j] + max(0.0, -min_cost);
		}
	}

	// order index assigned to each vehicle (-1 if not assigned an order)
	vector<int> assignment(cost_mat.size(), -1);
	if (int(cost_mat.size()) > 0 && int(cost_mat[0].size()) > 0){
		// Hungarian Assignment
		// HungarianAlgorithm HungAlgo;
		double cost = HUN_ASSIGN(cost_mat, assignment);
	}
	stop = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
	if (VERBOSITY == -1)
		cout << "hungarian_time," << duration.count() << endl;
	
    start = std::chrono::high_resolution_clock::now();
    
	for(int i = 0; i < int(assignment.size()); i++){
		if(assignment[i] != -1){
			if (cost_mat[i][assignment[i]] + FP_EPSILON >= (max_cost_val + max(0.0, -min_cost)))
				assignment[i] = -1;
		}
	}

	// Perform respective assignments
	unordered_map<int, int> vehicle_assigned;
	for(int i = 0; i < int(assignment.size()); i++){
		if (assignment[i] != -1 && best_plans[i][assignment[i]].first.first.size()!=0){
			all_vehicles[active_vehicles[i]].assign_order_pack(order_packs[assignment[i]],
															best_plans[i][assignment[i]].first.first, global_time);

			if (VERBOSITY > 0){
				for (auto ord_obj:order_packs[assignment[i]])
					cout << "Vehicle :" << all_vehicles[active_vehicles[i]].vehicle_id << " | Order : " <<  ord_obj.order_id << endl;
			}
			
			if (VERBOSITY == -1){
				for (auto ord_obj:order_packs[assignment[i]]){
					 vehicle ass_vh = all_vehicles[active_vehicles[i]];
					 cout << fixed << "ASSIGN,"<< ord_obj.order_id << "," << ass_vh.vehicle_id << ","<< global_time << ",";
					 cout<< nodes_to_latlon[ass_vh.path[ass_vh.path_present_idx]].first << "," << nodes_to_latlon[ass_vh.path[ass_vh.path_present_idx]].second << ",";
					 cout<< ord_obj.restaurant.rest_latlon << "," << ord_obj.customer.cust_latlon << endl;
					//cout << fixed << "ASSIGN,"<< ord_obj.order_id << "," << all_vehicles[active_vehicles[i]].vehicle_id << ","<< global_time << endl;
				}
			}
			vehicle_assigned[assignment[i]] = i;
		}
	}

	// fill rejected orders in this round
	for (int i = 0; i < int(order_packs.size()); i++){
		if (vehicle_assigned.find(i)==vehicle_assigned.end()){
			for (int j = 0; j < int(order_packs[i].size()); j++){
				rejected_orders.push_back(order_packs[i][j]);
			}
		}
	}
    stop = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
	if (VERBOSITY == -1)
		cout << "assignment_time," << duration.count() << endl;
}

vector<best_plan_tuple> pack_orders_cluster_gen_rp_opt_general_sp_as_AR_fair(vector<order> &order_set, double global_time, int vh_size){
	cout << "PACKING STARTED" << endl;
	
	double max_merge_cost = max_merge_cost_edt;

    if (max_merge_cost + FP_EPSILON >= MAX_NUM){
        throw "SOME PROBLEM HERE \n";
    }
	auto start = std::chrono::high_resolution_clock::now();

	int n = order_set.size();

	if (n == 0)
		return {};

	DSU dsu;
	dsu.init(n);
	
	vector<vector<best_plan_tuple>> order_graph = generate_start_order_undirected_graph_sp_as(order_set, global_time);
	
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	if (VERBOSITY == -1)
		cout << "cluster_pre_time," << duration.count() << endl;

	for(int i = 0; i < n; i++){
		for(int j = i+1; j < n; j++){
			double cost = order_graph[i][j].first.second;
			if (cost + FP_EPSILON >= MAX_NUM)
				continue;
		}
	}
	
	vector<best_plan_tuple> cluster_pack;
	for(auto it:dsu.set_to_elems){
		cluster_pack.push_back(order_graph[it.first][it.first]);
	}
	cout << "CLUSTER_SIZE = " << cluster_pack.size() << endl;
	cout << "CLUSTER_ITER = " << 0 << endl;
	return cluster_pack;
}

void two_SF(vector<int> &active_vehicles, vector<order> &active_orders, double global_time, vector<order> &rejected_orders){
	auto start = std::chrono::high_resolution_clock::now();

	int curr_time_slot = ((((long long int)(global_time))%86400 + 86400)%86400)/3600;

	// cout << global_time << " " << end_time + FP_EPSILON << endl;
	cout << "UNPICK ORDERS = " << active_orders.size() << endl;

	auto stop3 = std::chrono::high_resolution_clock::now();
	auto duration3 = std::chrono::duration_cast<std::chrono::microseconds>(stop3 - start);
	if (VERBOSITY == -1)
		cout << "collect_time," << duration3.count() << endl;

	vector<best_plan_tuple> bp_packs = pack_orders_cluster_gen_rp_opt_general_sp_as_AR_fair(active_orders, global_time, active_vehicles.size());
	
	vector<vector<order>> order_packs(bp_packs.size());
	for(int i = 0; i < int(bp_packs.size()); i++){
		for (auto ev:bp_packs[i].first.first){
			if (ev.type == 0)
				order_packs[i].push_back(ev.order_obj);
		}
	}
	
	auto stop2 = std::chrono::high_resolution_clock::now();
	auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - start);
	if (VERBOSITY == -1)
		cout << "cluster_time," << duration2.count() << endl;

	vector<vector<double>> cost_mat(active_vehicles.size(), vector<double>(order_packs.size(), MAX_NUM));
	vector<vector<pair<double,double>>> pay_times_mat(active_vehicles.size(), vector<pair<double,double>>(order_packs.size(), make_pair(0,0)));

	vector<vector<best_plan_tuple>> best_plans(active_vehicles.size(), vector<best_plan_tuple>(order_packs.size()));

	vector<vector<int>> node_to_order_pack(nodes_to_latlon.size());
	for (int i = 0; i < int(bp_packs.size()); i++)
		node_to_order_pack[bp_packs[i].first.first[0].node].push_back(i);
	
	int max_count_edges = int(double(active_orders.size())/double(active_vehicles.size()) * double(vehicle_explore_frac));
	max_count_edges = max(1, max_count_edges);

	double max_time_dist = slotted_max_time[curr_time_slot];
	
	double dj_time = 0.0;

	vector<int> pack_deg(bp_packs.size(), 0);

	vector<bool> visited(nodes_to_latlon.size(), false);
	vector<double> dist_vh(nodes_to_latlon.size(), MAX_NUM);
	vector<double> dist_ac(nodes_to_latlon.size(), MAX_NUM);

	// cout << global_time << " MJ " << global_conf.end_time + FP_EPSILON << endl;
	if ((global_time <= end_time + FP_EPSILON) && (active_orders.size() > 0)){
		cout << "AS IN AS" << endl;
		for(int vhi = 0; vhi < int(active_vehicles.size()); vhi++){
			if (bp_packs.size() == 0)
				continue;
			
			vehicle vh = all_vehicles[active_vehicles[vhi]];
			
			vector<event> org_route_plan = vh.route_plan;
			unordered_map<string, double> org_d_times = get_delivered_times(vh, org_route_plan, global_time);
			double org_cost;
            org_cost = get_route_plan_extra_delivery_time(org_d_times, org_route_plan);

			int count_edges = 0;

			fill(visited.begin(), visited.end(), false);
			fill(dist_vh.begin(), dist_vh.end(), MAX_NUM);
			fill(dist_ac.begin(), dist_ac.end(), MAX_NUM);
			long long int vh_start_node = vh.get_current_location();
			
			dist_vh[vh_start_node] = 0.0;
			dist_ac[vh_start_node] = 0.0;
			
			priority_queue<pair<double, long long int>> pq;

			// dist + h()
			pq.push({-dist_vh[vh_start_node], vh_start_node});

			double dist, alt;
			long long int u, v;

			auto dj_start = std::chrono::high_resolution_clock::now();
			while(!pq.empty()){
				pair<double, long long int> top = pq.top();
				pq.pop();

				u = top.second;

				if (visited[u])
					continue;
				
				if (count_edges > max_count_edges)
					break;

				if (dist_ac[u] > vehicle_rest_radius_cap){
					if (hhl_sp_query(vh_start_node, u) > vehicle_rest_radius_cap)
						break;
				}

				visited[u] = true;
				if (node_to_order_pack[u].size() != 0){
						for(auto pack_idx : node_to_order_pack[u]){
							if (count_edges > max_count_edges)
								break;
							
							double min_cost = MAX_NUM;
							vector<event> min_plan;
							unordered_map<string, double> min_delivery_times;

							vector<order> op = order_packs[pack_idx];
							vector<event> bp = bp_packs[pack_idx].first.first;
			
							if ((int(vh.order_set.size() + op.size()) > 1)) // no batching in 2SF
								continue;
							
							vector<vector<event>> all_route_plans = gen_inserted_sequence_rp(org_route_plan, bp);
							for(int ri = 0; ri < int(all_route_plans.size()); ri++){
								bool cap_compatible = check_capacity_constraint(vh, all_route_plans[ri]);

								unordered_map<string, double> d_times = get_delivered_times(vh, all_route_plans[ri], global_time);
								
								if (d_times.empty() || !cap_compatible)
									continue;

								bool sla_compatible = check_route_plan_sla_constraint(d_times, all_route_plans[ri]);

								if (!sla_compatible)
									continue;

								if (org_cost >= FP_EPSILON){
									cerr << "org_cost: " << org_cost << ", vh.order_set.size: " << vh.order_set.size()<< endl;
								}
								assert(org_cost < FP_EPSILON);

								
								double plan_cost;
                                plan_cost = get_route_plan_extra_delivery_time(d_times, all_route_plans[ri]) - org_cost;
								
								if (plan_cost < min_cost){
									min_cost = plan_cost;
									min_plan = all_route_plans[ri];
									min_delivery_times = d_times;
								}
							}
							cost_mat[vhi][pack_idx] = min_cost;
							best_plans[vhi][pack_idx] = {{min_plan, min_cost}, min_delivery_times};
							if (min_cost + FP_EPSILON < MAX_NUM)
								count_edges++;

						}
					//}
				}
				if (count_edges > max_count_edges)
					break;
				
				

				for(int ei = 0; ei < int(edges[u].size()); ei++){
					v = edges[u][ei];
					double e_weight = get_edge_weight(u, v, curr_time_slot);

					double heur_val = heuristic_function(vh, v);
					
					alt = dist_vh[u] + (1-heuristic_multiplier)*(e_weight)/max_time_dist + heuristic_multiplier*heur_val;
	
					if (alt < dist_vh[v]){
						dist_ac[v] = dist_ac[u] + e_weight;
						dist_vh[v] = alt;
						pq.push(make_pair(-alt, v));
					}
				}
			}
			auto dj_stop = std::chrono::high_resolution_clock::now();
			auto dj_duration = std::chrono::duration_cast<std::chrono::microseconds>(dj_stop - dj_start); 

			dj_time += dj_duration.count();
			
		}
	}
	cout << "dj_time," << dj_time << endl;

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
	if (VERBOSITY == -1)
		cout << "cost_time," << duration.count() << endl;

	start = std::chrono::high_resolution_clock::now();
	
	double min_cost = 0;
	for(auto it:cost_mat){
		for(auto it2:it)
			min_cost = min(min_cost, it2);
	}

	cout << "vehicle active time calculation" <<endl;
	for (int i=0; i<all_vehicles.size(); i++){
		vehicle* vh = &all_vehicles[i];
		double temp = 0;
		for (auto ai:vh->de_intervals){
			if(ai.end_time < global_time)
				temp += (ai.end_time - ai.start_time);
			else
				temp += max((global_time - ai.start_time), 0.0);
		}
		vh->active_time_AR = temp;
	}

	cout << "max pay calculation" << endl;
	double max_pay = FP_EPSILON;
	double max_time = FP_EPSILON;
	for (auto vh:all_vehicles){

		double vh_pay = 0.8 * vh.wait_time_AR + vh.travel_time_AR;
		if (vh_pay > max_pay){
			max_pay = vh_pay;
			max_time = vh.active_time_AR;
		}
	}

	cout << "cost matrix computation" << endl;
    //TODO make lambda_fair a command line argument
	double lambda_fair = 0.98;
	cout << "lambda_fair = " << lambda_fair << endl ;

	for(int i = 0; i < int(active_vehicles.size()); i++){
		vehicle vh_i = all_vehicles[active_vehicles[i]];
		double old_pay_i = 0.8 * vh_i.wait_time_AR + vh_i.travel_time_AR;

		for(int j = 0; j < int(order_packs.size()); j++){

			cost_mat[i][j] = (cost_mat[i][j] > max_cost_val)?max_cost_val:cost_mat[i][j];
			cost_mat[i][j] = cost_mat[i][j] + max(0.0, -min_cost);

			vector<event> rp_j = bp_packs[j].first.first;
			unordered_map<string, double> delivered_time = get_delivered_times(vh_i, rp_j, global_time);
			pair<double,double> pay_times =  get_pay_times_AR(delivered_time, rp_j);

			pay_times_mat[i][j].first = pay_times.first;
			pay_times_mat[i][j].second = pay_times.second;

			double pay_i_j = 0.8 * pay_times.first + pay_times.second;
			double time_i_j = vh_i.active_time_AR + pay_times.first + pay_times.second;
			if (cost_mat[i][j] < max_cost_val - FP_EPSILON)
				cost_mat[i][j] = (1 - lambda_fair) * cost_mat[i][j] + (lambda_fair) * abs(max_pay/max_time - (old_pay_i + pay_i_j)/time_i_j);
		}
	}



	cout<< "COST_MAT_MAXPAY,"<<max_pay<<",max_time,"<<max_time <<endl;
	
	// order index assigned to each vehicle (-1 if not assigned an order)
	cout << "HUN matching" << endl;
	vector<int> assignment(cost_mat.size(), -1);
	// cerr << "Hungarian matching" << endl;
	if (int(cost_mat.size()) > 0 && int(cost_mat[0].size()) > 0){
		// Hungarian Assignment
		double cost = HUN_ASSIGN(cost_mat, assignment);
	}
	stop = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
	if (VERBOSITY == -1)
		cout << "hungarian_time," << duration.count() << endl;
	
	for(int i = 0; i < int(assignment.size()); i++){
		if(assignment[i] != -1){
			if (cost_mat[i][assignment[i]] + FP_EPSILON >= (max_cost_val + max(0.0, -min_cost)))
				assignment[i] = -1;
		}
	}
	
	for(int i = 0; i < int(assignment.size()); i++){
		if (assignment[i] == -1)
			continue;

		vehicle* vh_i = &all_vehicles[active_vehicles[i]];
		vh_i->wait_time_AR += pay_times_mat[i][assignment[i]].first;
		vh_i->travel_time_AR += pay_times_mat[i][assignment[i]].second;
	}
	// Perform respective assignments
	unordered_map<int, int> vehicle_assigned;
	for(int i = 0; i < int(assignment.size()); i++){
		if (assignment[i] != -1){
			all_vehicles[active_vehicles[i]].assign_order_pack(order_packs[assignment[i]],
															best_plans[i][assignment[i]].first.first, global_time);

			if (VERBOSITY > 0){
				for (auto ord_obj:order_packs[assignment[i]])
					cout << "Vehicle :" << all_vehicles[active_vehicles[i]].vehicle_id << " | Order : " <<  ord_obj.order_id << endl;
			}
			
			if (VERBOSITY == -1){
				for (auto ord_obj:order_packs[assignment[i]]){
					vehicle ass_vh = all_vehicles[active_vehicles[i]];
					cout << fixed << "ASSIGN,"<< ord_obj.order_id << "," << ass_vh.vehicle_id << ","<< global_time << ",";
					cout<< nodes_to_latlon[ass_vh.path[ass_vh.path_present_idx]].first << "," << nodes_to_latlon[ass_vh.path[ass_vh.path_present_idx]].second << ",";
					cout<< ord_obj.restaurant.rest_latlon << "," << ord_obj.customer.cust_latlon << endl;
				}
			}
			vehicle_assigned[assignment[i]] = i;
		}
	}

	// fill rejected orders in this round
	for (int i = 0; i < int(order_packs.size()); i++){
		if (vehicle_assigned.find(i)==vehicle_assigned.end()){
			for (int j = 0; j < int(order_packs[i].size()); j++){
				rejected_orders.push_back(order_packs[i][j]);
			}
		}
	}
}
