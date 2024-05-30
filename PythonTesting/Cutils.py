import ctypes

import numpy as np
robotLib = ctypes.CDLL('cmake-build-debug/libRobotLib.dylib')

class matrix(ctypes.Structure):
    _fields_ = [
        ('numRows', ctypes.c_uint8),
        ('numCols', ctypes.c_uint8),
        ('data', ctypes.POINTER(ctypes.POINTER(ctypes.c_double))),
        ('square', ctypes.c_uint8)
    ]
class rigidBody(ctypes.Structure):
    _fields_ = [
        ('name', ctypes.c_char_p),
        ('mass', ctypes.POINTER(matrix)),
        ('Transform', ctypes.POINTER(matrix)),
        ('CoM', ctypes.POINTER(matrix))

    ]  # Example fields

class flexBody(ctypes.Structure):
    _fields_ = [
        ('mass', ctypes.POINTER(matrix)),#6x6 mass matrix
        ('transform', ctypes.POINTER(matrix)),#R6 transformation from start to end
        ('stiff', ctypes.POINTER(matrix)),#6x6 stiffness matrix
        ('damping', ctypes.POINTER(matrix)),#6x6 damping matrix
        ('F_0', ctypes.POINTER(matrix)),#R6 force at start
        ('N', ctypes.c_int),#number of elements to discretize the continuum todo some of these ints could be uint_8 or 16 to save memory
        ('L', ctypes.c_double),#length of the continuum
        ('eta_prev', ctypes.POINTER(matrix)),#6xN matrix of previous eta values
        ('eta_pprev', ctypes.POINTER(matrix)),#6xN matrix of previous eta values
        ('f_prev', ctypes.POINTER(matrix)),#6xN matrix of previous f values
        ('f_pprev', ctypes.POINTER(matrix)),#6xN matrix of previous f values
        ('CoM', ctypes.POINTER(matrix)),#R6 transformation start to COM todo is start J or I?
    ]  # Example fields

class Body_u(ctypes.Structure):
    _fields_ = [
        ('rigid', ctypes.POINTER(rigidBody)),
        ('flex', ctypes.POINTER(flexBody))
    ]
class Body(ctypes.Structure):
    _fields_ = [
        ('type', ctypes.c_uint8),
        ('body', ctypes.POINTER(Body_u))
    ]
class rigidJoint(ctypes.Structure):
    _fields_ = [
        ('name', ctypes.c_char_p),
        ('twistR6', ctypes.POINTER(matrix)),
        ('position', ctypes.c_double),
        ('velocity', ctypes.c_double),
        ('acceleration', ctypes.c_double),
        ('limits', ctypes.POINTER(ctypes.c_double)),
        ('homepos', ctypes.c_double),
        ('parent', ctypes.POINTER(Body)),
        ('child', ctypes.POINTER(Body))

    ]  # Example fields



class Object_u(ctypes.Structure):
    _fields_ = [
        ('name', ctypes.c_char_p),
        ('rigid', ctypes.POINTER(rigidBody)),
        ('flex', ctypes.POINTER(flexBody)),
        ('joint', ctypes.POINTER(rigidJoint))]

class Object(ctypes.Structure):
    _fields_ = [('type', ctypes.c_uint8),
                ('object', ctypes.POINTER(ctypes.POINTER(Object_u)))]

# Define the Robot structure
class Robot(ctypes.Structure):
    _fields_ = [
        ('name', ctypes.c_char_p),
        ('numObjects', ctypes.c_int),
        ('objects', ctypes.POINTER(ctypes.POINTER(Object)))
    ]


class ODE_output(ctypes.Structure):
    _fields_ = [
        ("eta_s", ctypes.POINTER(matrix)),
        ("f_s", ctypes.POINTER(matrix))
    ]
#________________-Functions-____________________





def makeMatrix(m):
    try:
        rows = m.shape[0]
    except IndexError:
        rows = 1
    try:
        cols = m.shape[1]
    except IndexError:
        cols = 1

    # Create row pointers
    row_pointers = (ctypes.POINTER(ctypes.c_double) * rows)()

    for i in range(rows):
        # Convert each row to a pointer to doubles
        try:
            row_pointers[i] = m[i, :].ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        except IndexError:
            row_pointers[i] = m.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

    # Create a Matrix instance
    matrix_c = matrix()
    matrix_c.numRows = ctypes.c_uint8(int(rows))
    matrix_c.numCols = ctypes.c_uint8(int(cols))
    matrix_c.data = row_pointers
    matrix_c.square = 1 if rows == cols else 0

    return matrix_c
def makeOdeOutput():
    # Create an instance of ODE_output
    ode_output = ODE_output()
    ode_output.eta_s = ctypes.pointer(makeMatrix(np.zeros((6, 1))))
    ode_output.f_s = ctypes.pointer(makeMatrix(np.zeros((6, 1))))

    return ode_output