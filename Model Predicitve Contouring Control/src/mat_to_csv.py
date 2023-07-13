import scipy.io
import pandas as pd
import pdb
mat = scipy.io.loadmat('track2.mat') 
mat = {k:v for k, v in mat.items() if k[0] != '_'}
pdb.set_trace()
data = pd.DataFrame({k: pd.Series(v[0]) for k, v in mat.iteritems()})
pdb.set_trace()
#data.to_csv("fiename.csv")