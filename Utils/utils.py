import pandas as pd
from IPython.core.display_functions import display


def print_df(df):
    with pd.option_context('display.max_rows', None, 'display.max_columns', None):
        display(df)