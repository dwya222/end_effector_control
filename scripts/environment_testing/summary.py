import pandas as pd
import numpy as np
import dataframe_image as dfi

position = 'p3'

df = pd.read_csv('origin_at_p3(0.15,0.5,-0.75)_.csv')
df.reset_index(drop=True, inplace=True)
df = df.rename(columns = {'center': 'back'})
df = df.drop(columns='Unnamed: 0')

def summarize(row):
    if (row['front'] == True or row['left'] == True or
        row['right'] == True or row['back'] == True):
            return 'Success'
    else:
        return 'Fail'


df['summary'] = df.apply(lambda row: summarize(row), axis=1)

df.to_csv(position+'_summary.csv')

amount_front = (len(df[df['front'] == True]))/12 * 100
amount_left = (len(df[df['left'] == True]))/12 * 100
amount_right = (len(df[df['right'] == True]))/12 * 100
amount_back = (len(df[df['back'] == True]))/12 * 100
amount_success = (len(df[df['summary'] == 'Success']))/12 * 100

# this should be a list of all the above values
df.loc[12] = ['%.1f%%' % amount_front, '%.1f%%' % amount_left,
              '%.1f%%' % amount_right, '%.1f%%' % amount_back,
              '%.1f%%' % amount_success]

df.insert(0, 'id', ['cup1','cup2','cup3','cup4','cup5','cup6','cup7','cup8',
                    'cup9','cup10','cup11','cup12', 'totals'])
print(df)

df_styled = df.style.background_gradient() #adding a gradient based on values in cell
dfi.export(df_styled,position+'.png')
