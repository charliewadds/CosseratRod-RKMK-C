import matlab.engine
import numpy as np
from numpy.matlib import rand
print("starting matlab")
eng = matlab.engine.start_matlab()
print("matlab started")
eng.cd(r'/Users/charliewadds/MATLAB/github/MultiBody-Dynamics-LG/Library', nargout=0)
eng.addpath(r'/Users/charliewadds/MATLAB/github/MultiBody-Dynamics-LG/Library', nargout=0)

arr = np.asarray(eng.Ad(rand(4,4), nargout=1))


print(arr)