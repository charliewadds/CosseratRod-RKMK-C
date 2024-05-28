import ctypes
from Cutils import *
import numpy as np

matrixLib = ctypes.CDLL('cmake-build-debug/libMatrixLib.dylib')
lieGroupLib = ctypes.CDLL('cmake-build-debug/libLieGroup.dylib')

#todo set argtypes
lieGroupLib.hat_R3.argtypes = [
    ctypes.POINTER(matrix),
    ctypes.POINTER(matrix)# result

]
lieGroupLib.hat_R3.restype = ctypes.POINTER(matrix)

class lieGroupCall:
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


    def call_adj(v, result_ptr):

        v_ptr = makeMatrix(v)

        lieGroupLib.adj(v_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[i][j]

        return result_numpy

    def call_adj_r6(v, result_ptr):

        v_ptr = makeMatrix(v)

        lieGroupLib.adj_R6(v_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[i][j]

        return result_numpy

    def call_expm_SO3(v, result_ptr):

            v_ptr = makeMatrix(v)

            lieGroupLib.expm_SO3(v_ptr, ctypes.byref(result_ptr))

            result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
            for i in range(result_ptr.numRows):
                for j in range(result_ptr.numCols):
                    result_numpy[i, j] = result_ptr.data[i][j]

            return result_numpy

    def call_expm_SE3(v, result_ptr):

        v_ptr = makeMatrix(v)

        lieGroupLib.expm_SE3(v_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[i][j]

        return result_numpy


class matrixCall:
    def call_matrix_add(m1, m2, result_ptr):
        m1_ptr = makeMatrix(m1)
        m2_ptr = makeMatrix(m2)

        matrixLib.matrix_add(m1_ptr, m2_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[i][j]

        return result_numpy


    def call_matrix_add3(m1, m2, m3, result_ptr):
        m1_ptr = makeMatrix(m1)
        m2_ptr = makeMatrix(m2)
        m3_ptr = makeMatrix(m3)

        matrixLib.matrix_add3(m1_ptr, m2_ptr, m3_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[i][j]

        return result_numpy

    def call_matrix_sub(m1, m2, result_ptr):
        m1_ptr = makeMatrix(m1)
        m2_ptr = makeMatrix(m2)

        matrixLib.matrix_sub(m1_ptr, m2_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[i][j]

        return result_numpy

    def call_matrix_scalar_mult(m, scalar, result_ptr):
        m_ptr = makeMatrix(m)

        matrixLib.matrix_scalar_mult(m_ptr, scalar, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[i][j]

        return result_numpy

    def call_matrix_solve(A, B, result_ptr):
        A_ptr = makeMatrix(A)
        B_ptr = makeMatrix(B)

        matrixLib.matrix_solve(A_ptr, B_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)

        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[i][j]

        return result_numpy

