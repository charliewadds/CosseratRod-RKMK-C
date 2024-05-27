import ctypes
from Cutils import *
import numpy as np

matrixLib = ctypes.CDLL('cmake-build-debug/libMatrixLib.dylib')
lieGroupLib = ctypes.CDLL('cmake-build-debug/libLieGroup.dylib')


lieGroupLib.hat_R3.argtypes = [
    ctypes.POINTER(matrix),
    ctypes.POINTER(matrix)# result

]
lieGroupLib.hat_R3.restype = ctypes.POINTER(matrix)
def call_hatR3(v, result_ptr):

    v_ptr = makeMatrix(v)

    result_1 = lieGroupLib.hat_R3(v_ptr, ctypes.byref(result_ptr))
    result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
    for i in range(result_ptr.numRows):
        for j in range(result_ptr.numCols):
            result_numpy[i, j] = result_ptr.data[i][j]

    return result_numpy
def call_hatR6(v, result_ptr):

    v_ptr = makeMatrix(v)

    result_1 = lieGroupLib.hat_R6(v_ptr, ctypes.byref(result_ptr))
    result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
    for i in range(result_ptr.numRows):
        for j in range(result_ptr.numCols):
            result_numpy[i, j] = result_ptr.data[i][j]

    return result_numpy


def call_unhatSO3(v, result_ptr):

    v_ptr = makeMatrix(v)

    lieGroupLib.unhat_SO3(v_ptr, ctypes.byref(result_ptr))

    result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
    for i in range(result_ptr.numRows):
        for j in range(result_ptr.numCols):
            result_numpy[i, j] = result_ptr.data[i][j]

    return result_numpy