import argparse
import numpy as np
import pandas as pd
import pytz
import datetime
import os
import re
import time
from datetime import datetime
import random
import copy
import itertools
from collections import Counter
import warnings

warnings.filterwarnings("ignore")

def gini(arr):
    ## first sort
    sorted_arr = arr.copy()
    sorted_arr.sort()
    n = arr.size
    coef_ = 2. / n
    const_ = (n + 1.) / n
    weighted_sum = sum([(i+1)*yi for i, yi in enumerate(sorted_arr)])
    return coef_*weighted_sum/(sorted_arr.sum()) - const_

parser = argparse.ArgumentParser()
parser.add_argument("--input_file", type=str, required=True)
parser.add_argument("--output_file", type=str, required=True)
parser.add_argument("--start", type=int, default=0)
parser.add_argument("--end", type=int, default=28)
parser.add_argument("--delta", type=int, default=180)
args = parser.parse_args()

delta = args.delta

result_file = args.input_file
output_file = args.output_file
st_hr = args.start
end_hr = args.end

avg_time = 0
avg_hungarian_time = 0
avg_cost_time = 0
overflow = 0
totalslots = 0

st = st_hr*3600
end = end_hr*3600

data = pd.read_csv(result_file, names=["a", "b", "c", "d", "e", "f", "g", "h"])

data_sdt = data[data['a'] == "SDT"].drop(['a', 'g', 'h'], axis = 1)
data_sdt.columns = ['order_id', 'sdt', 'ordered_time', 'prep_time', 'sla']
data_sdt = data_sdt.set_index('order_id')
data_sdt = data_sdt[(data_sdt.ordered_time >= st) & (data_sdt.ordered_time <= end)]

data_assign = data[data['a'] == "ASSIGN"].drop(['a', 'c', 'e', 'f', 'g', 'h'], axis = 1)
data_assign.columns = ['order_id', 'assigned_time']
data_assign = data_assign.groupby('order_id').max()

data_reject = data[data['a'] == 'REJECT'].drop(['a', 'c', 'd', 'e', 'f', 'g', 'h'], axis = 1)
data_reject.columns = ['order_id']

data_deliver = data[data['a'] == "DELIVER"].drop(['a', 'e', 'f', 'g', 'h'], axis = 1)
data_deliver.columns = ['order_id', 'delivered_time', 'vehicle_id']
data_deliver = data_deliver.set_index('order_id')

data_picked = data[data['a'] == "PICKEDUP"].drop(['a', 'd', 'e', 'f', 'g', 'h'], axis = 1)
data_picked.columns = ['order_id', 'picked_time']
data_picked = data_picked.set_index('order_id')

data_reached = data[data['a'] == "REACHED"].drop(['a', 'd', 'e', 'f', 'g', 'h'], axis = 1)
data_reached.columns = ['order_id', 'reached_time']
data_reached = data_reached.groupby('order_id').min()

data_move = data[data['a'] == "MOVE"].drop(['a'], axis = 1)
data_move.columns = ['vehicle_id', 'to_order', 'curr_time', 'dist_travelled',
                        'carrying_orders', 'time_travelled', 'event']
data_move['to_order'] = data_move['to_order'].astype('int')

data_idle = data[data['a'] == "IDLE"].drop(['a', 'e', 'f', 'g', 'h'], axis = 1).astype('float64')
data_idle.columns = ['vehicle_id', 'curr_time', 'time_stay_idle']

df = pd.concat([data_sdt, data_deliver, data_picked, data_reached, data_assign], join='inner', axis=1)

df = df[(df.ordered_time >= st) & (df.ordered_time <= end)]

data_cost_time = data[data['a'] == "cost_time"].drop(['a', "c", "d", "e", "f", "g", "h"], axis = 1).astype('float64')
data_cost_time.columns = ['cost_time']
data_hung_time = data[data['a'] == "hungarian_time"].drop(['a', "c", "d", "e", "f", "g", "h"], axis = 1).astype('float64')
data_hung_time.columns = ['hungarian_time']
avg_time_taken_per_slot = (sum(data_hung_time['hungarian_time'])+sum(data_cost_time['cost_time']))/(data_cost_time.shape[0]*1000000)
avg_time += avg_time_taken_per_slot
avg_hungarian_time += sum(data_hung_time['hungarian_time'])
avg_cost_time += sum(data_cost_time['cost_time'])
assign_time = (data_cost_time.reset_index()['cost_time']+data_hung_time.reset_index()['hungarian_time'])/1000000
overflow += len(assign_time[assign_time > 180])
totalslots += len(assign_time)
avg_time
overflow_percent = 100*overflow/totalslots

data_move = data_move[(data_move.curr_time >= st) & (data_move.curr_time <= end)]
data_all = df
data_move_all = data_move
data_idle_all = data_idle

DE_travelled = data_move_all.dropna().groupby(['vehicle_id']).agg({'dist_travelled' : 'sum', 'time_travelled' : 'sum'}).reset_index()
DE_idle = data_idle_all.dropna().groupby(['vehicle_id']).agg({'time_stay_idle' : 'sum'}).reset_index()
df_DE_foodmatch = data_all.reset_index()[['order_id', 'vehicle_id', 'delivered_time','picked_time', 'reached_time']].dropna()
df_DE_foodmatch['de_wait_time'] = df_DE_foodmatch.apply(lambda row: (datetime.fromtimestamp(row.picked_time) - datetime.fromtimestamp(row.reached_time)).seconds / 60 , axis=1)
DE_wait = df_DE_foodmatch.groupby(['vehicle_id']).agg({'de_wait_time' : 'sum'})
data_final_foodmatch = pd.merge(DE_travelled, DE_wait, left_on=['vehicle_id'], right_on = ['vehicle_id'])
data_final_foodmatch2 = pd.merge(data_final_foodmatch, DE_idle, left_on=['vehicle_id'], right_on = ['vehicle_id'])
data_final_foodmatch2['ACTIVE_MINUTE'] = data_final_foodmatch2.apply(lambda row:  ((row.time_travelled+ row.time_stay_idle)/60 + row.de_wait_time) , axis=1)
data_final_foodmatch3 = data_final_foodmatch2[data_final_foodmatch2['time_travelled']/60 + data_final_foodmatch2['de_wait_time'] < data_final_foodmatch2['ACTIVE_MINUTE']]
data_final_foodmatch2["time_present"]=data_final_foodmatch2['time_travelled']/60 + data_final_foodmatch2['de_wait_time']
data_final_foodmatch3['pay_per_active_hour'] = data_final_foodmatch3.apply(lambda row: ((row.time_travelled/60) + 0.8*(row.de_wait_time))/ (row.ACTIVE_MINUTE) , axis=1)
data = np.sort(np.array(data_final_foodmatch3['pay_per_active_hour']))
gini_pay_per_active_hr = gini(data)
data_final_foodmatch2['pay'] = data_final_foodmatch2.apply(lambda row: ((row.time_travelled/60) + 0.8*(row.de_wait_time)) , axis=1)
df_DE_foodmatch = data_all.reset_index()[['order_id', 'vehicle_id', 'delivered_time','picked_time', 'reached_time','ordered_time']].dropna()
df_DE_foodmatch['order_delivered_time'] = df_DE_foodmatch.apply(lambda row: (datetime.fromtimestamp(row.delivered_time) - datetime.fromtimestamp(row.ordered_time)).seconds / 60 , axis=1)
orders_assigned = df_DE_foodmatch['order_id'].nunique()
orders_delivered_45min = df_DE_foodmatch[df_DE_foodmatch['order_delivered_time'] < 45]['order_id'].nunique()
total_delivery_time_fair = sum(df_DE_foodmatch['order_delivered_time'])
total_payment_fair = sum(data_final_foodmatch2['pay'])
pay_pah_min60 =  max(data_final_foodmatch3[data_final_foodmatch3['ACTIVE_MINUTE']>60]['pay_per_active_hour']) - min(data_final_foodmatch3[data_final_foodmatch3['ACTIVE_MINUTE']>60]['pay_per_active_hour'])
orders_rejected = data_reject['order_id'].nunique()
total_orders = orders_assigned + orders_rejected
SLA_voilation = 100 * (total_orders - orders_delivered_45min)/total_orders

vh_orders = df_DE_foodmatch.groupby(['vehicle_id']).agg({'order_id' : 'nunique'}).reset_index()
result = "Gini,SLA Violation,DTPO,Payment Gap, Avg. Runtime Per Window, % of overflow window, 25 percentile,50 percentile,75 percentile,95 percentile\n"+str(gini_pay_per_active_hr)+","+str(SLA_voilation)+","+str(total_delivery_time_fair/orders_assigned)+","+str(pay_pah_min60*60)+","+str(avg_time)+","+str(overflow_percent)+","+str(vh_orders.order_id.quantile(0.25))+","+str(vh_orders.order_id.quantile(0.50))+","+str(vh_orders.order_id.quantile(0.75))+","+str(vh_orders.order_id.quantile(0.95))+"\n"
#print(result)
print("Gini : ",gini_pay_per_active_hr)
print("SLA Violation : ",SLA_voilation)
print("DTPO : ",total_delivery_time_fair/orders_assigned)
print("Payment Gap : ",pay_pah_min60*60)
print("Avg. Runtime Per Window : ", avg_time," Sec")
print("% of overflow window : ", overflow_percent)
print("Number of Orders per Driver at 25 percentile : ", vh_orders.order_id.quantile(0.25))
print("Number of Orders per Driver at 50 percentile : ", vh_orders.order_id.quantile(0.50))
print("Number of Orders per Driver at 75 percentile : ", vh_orders.order_id.quantile(0.75))
print("Number of Orders per Driver at 95 percentile : ", vh_orders.order_id.quantile(0.95))

f= open(output_file,"a+")
f.write(result)
f.close()



