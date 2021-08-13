#!/usr/bin/env python3

import pandas as pd
import dataframe_image as dfi
import os

minimal_success_rate_list = []
overall_success_rate_list = []

for i in range(1,8):
    df = pd.read_pickle("./position_" + str(i) + "_.pkl")
    overall_success_rate = df['grasp_summary'].loc[len(df)-1]
    df = df.drop(['front', 'left', 'right', 'back', 'grasp_summary'], axis=1)
    df = df[df['cup_summary'] != '-----']
    minimal_success_rate_float = float(len(df[df['cup_summary'] != '0.0%']))/len(df) * 100
    minimal_success_rate = '%.1f%%' %minimal_success_rate_float

    df = df.reset_index()
    df = df.drop('index', axis=1)
    for j in range(0,12):
        df['id'][j] = j

    df.columns = ['Cup #', 'Cup Success Rate']
    df['Overall Success Rate'] = ''
    df['Overall Success Rate'][0] = overall_success_rate
    df['Minimal Success Rate'] = ''
    df['Minimal Success Rate'][0] = minimal_success_rate
    print(df)
    print(overall_success_rate)
    print(minimal_success_rate)
    df_styled = df.style.background_gradient() #adding a gradient based on values in cell
    dfi.export(df_styled,'position_' + str(i) + '_.png')

    minimal_success_rate_list.append(minimal_success_rate)
    overall_success_rate_list.append(overall_success_rate)

positions = [1,2,3,4,5,6,7]
df = pd.DataFrame({'Positions': positions, 'Overall Success Rates': overall_success_rate_list,
                   'Minimal Success Rates': minimal_success_rate_list})

# df_styled = df.style.background_gradient() #adding a gradient based on values in cell
df.to_pickle('overall_results.pkl')
dfi.export(df,'overall_results_.png')
