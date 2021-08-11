#!/usr/bin/env python3

import pandas as pd
import dataframe_image as dfi
import os

df = pd.read_pickle("./position_1_1.pkl")
df_styled = summarized_df.style.background_gradient() #adding a gradient based on values in cell
dfi.export(df_styled,'position_1_1.png')
