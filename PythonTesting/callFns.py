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
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]

        return result_numpy
    def call_hatR6(v, result_ptr):

        v_ptr = makeMatrix(v)

        result_1 = lieGroupLib.hat_R6(v_ptr, ctypes.byref(result_ptr))
        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]
        return result_numpy


    def call_unhatSO3(v, result_ptr):

        v_ptr = makeMatrix(v)

        lieGroupLib.unhat_SO3(v_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]

        return result_numpy


    def call_adj(v, result_ptr):

        v_ptr = makeMatrix(v)

        lieGroupLib.adj(v_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]

        return result_numpy

    def call_adj_r6(v, result_ptr):

        v_ptr = makeMatrix(v)

        lieGroupLib.adj_R6(v_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]

        return result_numpy

    def call_expm_SO3(v, result_ptr):

            v_ptr = makeMatrix(v)

            lieGroupLib.expm_SO3(v_ptr, ctypes.byref(result_ptr))

            result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
            for i in range(result_ptr.numRows):
                for j in range(result_ptr.numCols):
                    result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]
            return result_numpy

    def call_expm_SE3(v, result_ptr):

        v_ptr = makeMatrix(v)

        lieGroupLib.expm_SE3(v_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]

        return result_numpy


class matrixCall:
    def call_matrix_add(m1, m2, result_ptr):
        m1_ptr = makeMatrix(m1)
        m2_ptr = makeMatrix(m2)

        matrixLib.matrix_add(m1_ptr, m2_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]

        return result_numpy


    def call_matrix_add3(m1, m2, m3, result_ptr):
        m1_ptr = makeMatrix(m1)
        m2_ptr = makeMatrix(m2)
        m3_ptr = makeMatrix(m3)

        matrixLib.matrix_add3(m1_ptr, m2_ptr, m3_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]

        return result_numpy

    def call_matrix_sub(m1, m2, result_ptr):
        m1_ptr = makeMatrix(m1)
        m2_ptr = makeMatrix(m2)

        matrixLib.matrix_sub(m1_ptr, m2_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]

        return result_numpy

    def call_matrix_scalar_mult(m, scalar, result_ptr):
        m_ptr = makeMatrix(m)

        matrixLib.matrix_scalar_mult(m_ptr, scalar, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)
        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]

        return result_numpy

    def call_matrix_solve(A, B, result_ptr):
        A_ptr = makeMatrix(A)
        B_ptr = makeMatrix(B)

        matrixLib.matrix_solve(A_ptr, B_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)

        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]

        return result_numpy

    def call_matrix_mult(A, B, result_ptr):
        A_ptr = makeMatrix(A)
        B_ptr = makeMatrix(B)

        matrixLib.matMult(A_ptr, B_ptr, ctypes.byref(result_ptr))

        result_numpy = np.zeros((result_ptr.numRows, result_ptr.numCols), dtype=np.float64)

        for i in range(result_ptr.numRows):
            for j in range(result_ptr.numCols):
                result_numpy[i, j] = result_ptr.data[(i * result_ptr.numCols) + j]

        return result_numpy


class robotCall:
    def call_defPaperSample2(theta: ctypes.POINTER(matrix), theta_dot: ctypes.POINTER(matrix), theta_ddot: ctypes.POINTER(matrix)):
        robot_ptr = ctypes.POINTER(Robot)()
        robot_ptr = robotLib.defPaperSample_2(theta, theta_dot, theta_ddot)


        return robot_ptr

    def matrixToListStr(self, matrix: ctypes.POINTER(matrix)) -> str:
        result = '['
        for i in range(matrix.contents.numRows):
            for j in range(matrix.contents.numCols):
                result += str(matrix.contents.data[(i * matrix.contents.numCols) + j]) + ' '
            result += ']\n'
        return result
    def objectToDict(self, object: ctypes.POINTER(Object)) -> list:
        object_list = []
        if object.contents.type == 0:
            object_u = object.contents.object.contents
            rigid = object_u.rigid.contents
            object_dict = {
                'Name': object_u.name.decode('utf-8'),
                'Type': 'RIGID',
                'Mass': self.matrixToListStr(rigid.mass),
                'Transform': self.matrixToListStr(rigid.transform),
                'Stiff': self.matrixToListStr(rigid.stiffness),
                'Damp': self.matrixToListStr(rigid.damping),
                'CoM': self.matrixToListStr(rigid.CoM), # Add other required attributes similarly
                'F_0': self.matrixToListStr(rigid.F_0),


            }
            object_list.append(object_dict)
        elif object.contents.type == 1:
            object_u = object.contents.object.contents
            flex = object_u.flex.contents
            object_dict = {
                'Name': object_u.name.decode('utf-8'),
                'Type': 'FLEXIBLE',
                'Mass': self.matrixToListStr(flex.mass),  # Assuming these are similar to the 'Mass' in your provided format
                'Inertia': flex.inertia  # Add other required attributes similarly
            }
            object_list.append(object_dict)
        elif object.contents.type == 2:
            object_u = object.contents.object.contents
            joint = object_u.joint.contents
            object_dict = {
                'Name': object_u.name.decode('utf-8'),
                'Type': 'JOINT',
                'JointType': joint.type,  # Assuming these are similar to the 'Type' in your provided format
                'Axis': joint.axis  # Add other required attributes similarly
            }
            object_list.append(object_dict)


        return object_list
    def robotToMatlab(self, robot_ptr: ctypes.POINTER(Robot)):

        objects = []
        for i in range(robot_ptr.contents.numObjects):
            object = robot_ptr.contents.objects[i]
            if(object.contents.type == 0):
                object_u = object.contents.object.contents
                rigid = object_u.rigid.contents
                objects.append(rigid)

