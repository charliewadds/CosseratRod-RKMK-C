
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

    err = np.zeros(m1.shape)
    absErr = np.zeros(m1.shape)

    maxAbsErr = 0
    for i in range(m1.shape[0]):
        for j in range(m1.shape[1]):
            err[i, j] = m1[i, j] - m2['C_des'][i, j]
            absErr[i, j] = abs(err[i, j])
            if(absErr[i, j] > maxAbsErr):
                maxAbsErr = absErr[i, j]

    print("Max abs error: ", maxAbsErr)
    print("Mean abs error: ", np.mean(absErr))

    np.savetxt("./err.csv", err, delimiter=",", fmt='%.15f')
    np.savetxt("./absErr.csv", absErr, delimiter=",", fmt='%.15f')
    np.savetxt("./matlab2.csv", m2['C_des'], delimiter=",", fmt='%.15f')

compare_csv('cmake-build-debug/Control_good.csv', 'ControlSim2.mat')