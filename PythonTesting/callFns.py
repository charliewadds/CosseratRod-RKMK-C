import ctypes
from Cutils import *
import numpy as np

matrixLib = ctypes.CDLL('cmake-build-debug/libMatrixLib.dylib')
lieGroupLib = ctypes.CDLL('cmake-build-debug/libLieGroup.dylib')
robotLib = ctypes.CDLL('cmake-build-debug/libRobotLib.dylib')

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

    def call_matrix_mult(A, B, result_ptr):
        A_ptr = makeMatrix(A)
        B_ptr = makeMatrix(B)

        matrixLib.matMult(A_ptr, B_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)

        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[i][j]

        return result_numpy


class robotCall:
    def call_ode(eta, f, eta_h, f_h, f_sh, K, C, M, c0, f_0, Fd_ext, result):
        eta_ptr = makeMatrix(eta)
        f_ptr = makeMatrix(f)
        eta_h_ptr = makeMatrix(eta_h)
        f_h_ptr = makeMatrix(f_h)
        f_sh_ptr = makeMatrix(f_sh)
        K_ptr = makeMatrix(K)
        C_ptr = makeMatrix(C)
        M_ptr = makeMatrix(M)

        f_0_ptr = makeMatrix(f_0)
        Fd_ext_ptr = makeMatrix(Fd_ext)


        out = makeOdeOutput()

        result_ptr = robotLib.COSS_ODE(eta_ptr, f_ptr, eta_h_ptr, f_h_ptr, f_sh_ptr, K_ptr, C_ptr, M_ptr, ctypes.c_double(c0), f_0_ptr, Fd_ext_ptr, out)

# Now `result_ptr` is a pointer to ODE_output structure
        ode_output_ptr = ctypes.cast(result_ptr, ctypes.POINTER(ODE_output))

# Dereference the pointer to access the ODE_output structure
        ode_output_f_s = ctypes.cast(ode_output_ptr.contents.f_s, ctypes.POINTER(matrix))
        ode_output_eta_s = ctypes.cast(ode_output_ptr.contents.eta_s, ctypes.POINTER(matrix))

        result_numpy_f_s = np.zeros((ode_output_f_s.numRows, ode_output_f_s.numCols), dtype=np.float64)

        for i in range(ode_output_f_s.numRows):
            for j in range(result.numCols):
                result_numpy_f_s[i, j] = result.data[i][j]

        return