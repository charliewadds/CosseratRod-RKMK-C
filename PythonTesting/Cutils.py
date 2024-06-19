import ctypes
import matlab.engine
import numpy as np
robotLib = ctypes.CDLL('cmake-build-debug/libRobotLib.dylib')


print("starting matlab")
eng = matlab.engine.start_matlab()
print("matlab started")
eng.cd(r'/Users/charliewadds/MATLAB/projects/ASRoMGit/Library', nargout=0)
eng.addpath(r'/Users/charliewadds/MATLAB/projects/ASRoMGit//Library', nargout=0)

class matrix(ctypes.Structure):
    _fields_ = [
        ('numRows', ctypes.c_uint8),
        ('numCols', ctypes.c_uint8),
        ('data', ctypes.POINTER(ctypes.c_double)),
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


# typedef struct robot_s {
#     char *name;
# int numObjects;
# Object **objects;
# }Robot;
class Robot(ctypes.Structure):
    _fields_ = [
        ('name', ctypes.c_char_p),
        ('numObjects', ctypes.c_int),
        ('objects', ctypes.POINTER(ctypes.POINTER(Object)))

    ]

def makeRigidBodyFromMatlab(matlabRigidBody):
    rigid = rigidBody()
    rigid.name = ctypes.c_char_p(matlabRigidBody['Name'].encode('utf-8'))
    rigid.mass = ctypes.pointer(makeMatrix(np.asarray(matlabRigidBody['Mass'])))
    rigid.Transform = ctypes.pointer(makeMatrix(np.asarray(matlabRigidBody['Transform'])))
    rigid.CoM = ctypes.pointer(makeMatrix(np.asarray(matlabRigidBody['CoM'])))
    return rigid
def makeFlexBodyFromMatlab(matlabFlexBody):
    flex = flexBody()
    flex.mass = ctypes.pointer(makeMatrix(np.asarray(matlabFlexBody['Mass'])))
    flex.transform = ctypes.pointer(makeMatrix(np.asarray(matlabFlexBody['Transform'])))
    flex.stiff = ctypes.pointer(makeMatrix(np.asarray(matlabFlexBody['Stiff'])))
    flex.damping = ctypes.pointer(makeMatrix(np.asarray(matlabFlexBody['Damp'])))
    flex.F_0 = ctypes.pointer(makeMatrix(np.asarray(matlabFlexBody['F_0'])))
    flex.N = ctypes.c_int(int(matlabFlexBody['N']))
    flex.L = ctypes.c_double(matlabFlexBody['L'])
    flex.eta_prev = ctypes.pointer(makeMatrix(np.zeros((6, flex.N))))
    flex.eta_pprev = ctypes.pointer(makeMatrix(np.zeros((6, flex.N))))
    flex.f_prev = ctypes.pointer(makeMatrix(np.zeros((6, flex.N))))
    flex.f_pprev = ctypes.pointer(makeMatrix(np.zeros((6, flex.N))))
    flex.CoM = ctypes.pointer(makeMatrix(np.asarray(matlabFlexBody['CoM'])))
    return flex
def makeRigidJointFromMatlab(matlabRigidJoint, robot, i):
    joint = rigidJoint()
    joint.name = ctypes.c_char_p(matlabRigidJoint['Name'].encode('utf-8'))
    joint.twistR6 = ctypes.pointer(makeMatrix(np.asarray(matlabRigidJoint['Twist'])))
    joint.position = ctypes.c_double(matlabRigidJoint['Position'])
    joint.velocity = ctypes.c_double(matlabRigidJoint['Vel'])
    joint.acceleration = ctypes.c_double(matlabRigidJoint['Accel'])
    joint.limits = (np.asarray(matlabRigidJoint['Limit'])).ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    joint.homepos = ctypes.c_double(matlabRigidJoint['HomePos'])
    if(matlabRigidJoint['Parent']['Type'] == 0):
        body = Body()
        body.type = ctypes.c_uint8(1)
        bodyU = Body_u()
        bodyU.flex = robot.objects[0][i-1].object.contents.contents.flex
        body.body = ctypes.pointer(bodyU)
    else:
        body = Body()
        body.type = ctypes.c_uint8(1)
        bodyU = Body_u()
        bodyU.rigid = robot.objects[i-1].object.contents.contents.rigid
        body.body = ctypes.pointer(bodyU)

        joint.parent = ctypes.pointer(body)
    if(matlabRigidJoint['Child']['Type'] == 0):
        body = Body()
        body.type = ctypes.c_uint8(1)
        bodyU = Body_u()
        bodyU.flex = robot.objects[0][i+1].object.contents.contents.flex
        body.body = ctypes.pointer(bodyU)
        joint.child = ctypes.pointer(body)
    else:
        body = Body()
        body.type = ctypes.c_uint8(1)
        bodyU = Body_u()
        bodyU.rigid = robot.objects[0][i+1].object.contents.contents.rigid
        body.body = ctypes.pointer(bodyU)
        joint.child = ctypes.pointer(body)

    return joint

def makeObjectFromMatlab(matlabObject, robot, i):
    object = Object_u()
    if matlabObject['Type'] == 'RIGID' and matlabObject['Name'][0] != 'J':
        object.rigid = ctypes.pointer(makeRigidBodyFromMatlab(matlabObject))
    elif matlabObject['Type'] == 'FLEXIBLE':
        object.flex = ctypes.pointer(makeFlexBodyFromMatlab(matlabObject))
    else:
        object.joint = ctypes.pointer(makeRigidJointFromMatlab(matlabObject, robot, i))
    return object
def makeRobotFromMatlab():
    robot = Robot()
    theta = np.array([0, 0, 0, 0, 0])
    theta_dot = np.array([0, 0, 0, 0, 0])
    theta_ddot = np.array([0, 0, 0, 0, 0])

    matlabBot = eng.defPaperSample_2(theta, theta_dot, theta_ddot, nargout=1)

    #robot.name = ctypes.c_char_p(matlabBot['name'].encode('utf-8'))
    robot.numObjects = len(matlabBot)
    robot.objects = (ctypes.POINTER(Object) * robot.numObjects)()
    for i in range(robot.numObjects):
        object = Object()
        if(matlabBot[i]['Name'][0] != 'J'):
            object.object = ctypes.pointer(ctypes.pointer(makeObjectFromMatlab(matlabBot[i], robot, i)))
        robot.objects[i] = ctypes.pointer(object)

    for i in range(robot.numObjects):
        object = Object()
        if(matlabBot[i]['Name'][0] == 'J'):
            object.object = ctypes.pointer(ctypes.pointer(makeObjectFromMatlab(matlabBot[i], robot, i)))
        robot.objects[i] = ctypes.pointer(object)

    return robot
def makeMatrix(m):
    try:
        rows = m.shape[0]
    except IndexError:
        rows = 1

    try:
        cols = m.shape[1]
    except IndexError:
        cols = 1

    # Create a Matrix instance
    matrix_c = matrix()
    matrix_c.numRows = ctypes.c_uint8(int(rows))
    matrix_c.numCols = ctypes.c_uint8(int(cols))
    matrix_c.data = m.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    matrix_c.square = 1 if rows == cols else 0

    return matrix_c

def makeOdeOutput():
    # Create an instance of ODE_output
    ode_output = ODE_output()
    ode_output.eta_s = ctypes.pointer(makeMatrix(np.zeros((6, 1))))
    ode_output.f_s = ctypes.pointer(makeMatrix(np.zeros((6, 1))))

    return ode_output

print(makeRobotFromMatlab())