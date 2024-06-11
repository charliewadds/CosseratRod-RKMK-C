
from random import random

import matlab.engine
import numpy as np
import scipy

from termcolor import colored



import csv

import numpy as np



def compare_csv(file1, file2):
    m1 = np.loadtxt(file1, dtype=np.double, delimiter=",")
    m2 = scipy.io.loadmat(file2)


    diff = np.sum(np.subtract(m1, m2['C_des']))

    print(diff)
    print("M1")
    print(m1)
    print("M2")
    print(m2['C_des'])

    np.savetxt("./matlab2.csv", m2['C_des'], delimiter=",", fmt='%.15f')

compare_csv('cmake-build-debug/Control_good.csv', 'ControlSim2.mat')