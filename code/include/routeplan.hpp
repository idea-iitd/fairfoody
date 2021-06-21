#pragma once

#include <bits/stdc++.h>
#include "event.hpp"
#include "vehicle.hpp"

using namespace std;

// Generates all permutations of "route_plan" from l to r index and pushes them to "answer"
void generate_route_plans(vector<vector<event>> &answer, vector <event> &route_plan, int l, int r);

// Cost function
double get_route_plan_extra_delivery_time(unordered_map<string, double> &delivered_time, vector<event> &route_plan);

pair<double,double> get_pay_times_AR(unordered_map<string, double> &delivered_time, vector<event> &route_plan);

// Returns the time taken to deliver all the orders in route plan(sec)
// for all orders in delivered_time
double get_route_plan_total_delivery_time(unordered_map<string, double> &delivered_time, vector<event> &route_plan, double global_time);

// Given a vehicle and routeplan at time curr_time,
// returns the delivered times of orders in the routeplan if vehicle follows it
unordered_map<string, double> get_delivered_times(vehicle &vh, vector<event> &route_plan, double curr_time);

// Given a vehicle object and a route plan,
// returns if capacity constraint is satisfied while following the route plan
bool check_capacity_constraint(vehicle &vh, vector<event> &route_plan);


// returns if the delivered times follow the promised delivery time constraint
bool check_route_plan_sla_constraint(unordered_map<string, double> &delivered_time, vector<event> &route_plan);



// Given vehicle object, new order and current time(UNIX timestamp in sec)
// returns best route plan to follow to minimize extra delivery time
// returms ((route_plan, plan_cost), delivery_times)
pair<pair<vector<event>, double>, unordered_map<string, double>> get_best_route_plan(vehicle &vh, order &new_order,
                                                                                    double curr_time);
