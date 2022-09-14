import math
import pandas as pd


def str_to_bool(str_, strip_:bool=True, case_invariant:bool=True):
    """
    Converts string to boolean. If str_ is 'True' returns true, else False.

    Args:
        str_ (str): String to evaluate
        strip_ (bool): Whether to strip white space from string
        case_invariant (bool): Whether to evaluate other case types like 'true' as True

    Returns:
        boolean whether string is True
    """
    if not isinstance(str_,str):
        raise TypeError('Invalid type for str_to_bool {type(str_)}. Must be string.')

    if strip_ is True:
        str_ = str(str_).strip()
    if case_invariant is True:
        return str_.lower() == 'true'
    else:
        return str_ == 'True'

def df_compare_to_dict(df1:pd.DataFrame, df2:pd.DataFrame)->dict:
    """
    Return conslidated dictionary of differences between dataframes.
    Dictionary contains column names and values of different value of `df2`
    relative to `df`.

    dict_ = {index1: {'col1':val1_of_df2, 'col2':'val2_of_df2, ...}, ...}

    Args:
        df1 (pd.DataFrame): First dataframe to compare
        df2 (pd.DataFrame): Second dataframe to compare

    Returns:
        dictionary of differences between dataframes
    """
    # Compare differences
    diff = df1.compare(df2)
    # Get only 'other' columns (AKA columns of values from df2 differnt than df1)
    diff = diff.loc[:, (slice(None),'other')]
    # Remove swap the column name to data that changed from 'other' multi index column
    diff.columns = [col_group for col_group, col in diff.columns]
    # Dictioanry of differnces
    diff_dict = diff.to_dict('index')
    # Remove nan (no differences)
    for key, dict_ in diff_dict.items():
        del_keys = []
        for subkey, val in dict_.items():
            if math.isnan(val):
                del_keys.append([key, subkey])
        for del_key in del_keys:
            del diff_dict[del_key[0]][del_key[1]]
    return diff_dict