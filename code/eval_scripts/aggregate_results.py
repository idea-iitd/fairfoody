import sys
import pandas as pd
from glob import glob 
import os
import numpy as np

city = sys.argv[1]
algo = sys.argv[2]

files = sorted(glob("metrics/{}/*/{}*".format(city, algo)))

dfs = []
for f in files:
    df = pd.read_csv(f, names=['idx', 'filename', 'mean_xdt', 'wait_time', 'delivered', 'opkm', 'reject', 'avg_time', 'std_time', 'peak_vio'])
    dfs.append(df)

df = pd.concat(dfs)

print(len(df))
print("wait,%f" %df['wait_time'].mean())
print("opkm,%f" %df['opkm'].mean())
print("avg_time,%f" %df['avg_time'].mean())
print("XDT,%f" %((df['mean_xdt']*df['delivered'] + 120*df['reject']).sum()/(df['delivered'] + df['reject']).sum()))
print("avg time,%f" %df[df.peak_vio > -0.5]['avg_time'].mean())
print("peak vio,%f" %df[df.peak_vio > -0.5]['peak_vio'].mean())
print("reject,%f" %df['reject'].sum())
print("deli,%f" %df['delivered'].sum())
