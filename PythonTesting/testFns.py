from random import random

import matlab.engine
import numpy as np

from termcolor import colored

print("starting matlab")
eng = matlab.engine.start_matlab()
print("matlab started")
eng.cd(r'/Users/charliewadds/MATLAB/projects/ASRoMGit/Library', nargout=0)
eng.addpath(r'/Users/charliewadds/MATLAB/projects/ASRoMGit//Library', nargout=0)

from callFns import *

class testLieGroup:
    def test_hatR3(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        #v = np.array([[], [2], [3]], dtype=np.float64)
        v = np.random.rand(3,1) * 10
        # Call the MATLAB function
        matlab_result = matlab_engine.hat(v)
        matlab_result = np.array(matlab_result)
        npResult = np.zeros((3,3))
        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = lieGroupCall.call_hatR3(v,c_result)


        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||")
        print("testing hatR3 in C against hatR3 in MATLAB")
        print("------------------------------------------------------")
        print("\n\t\t\t\tInput")
        print(v)
        print("\n\t\t\t\tMATLAB Output")
        print(matlab_result)

        print("\n\t\t\t\tC Output")
        print(c_result)

        np.testing.assert_allclose(c_result, matlab_result, rtol=1e-5, atol=1e-8)
        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
    def test_hatR6(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        v = np.random.rand(6,1) * 10

        # Call the MATLAB function
        matlab_result = matlab_engine.hat(v)
        matlab_result = np.array(matlab_result)

        npResult = np.zeros((4,4))
        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = lieGroupCall.call_hatR6(v,c_result)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||")
        print("testing hatR6 in C against hatR6 in MATLAB")
        print("------------------------------------------------------")
        print("\n\t\t\t\tInput")
        print(v)
        print("\n\t\t\t\tMATLAB Output")
        print(matlab_result)

        print("\n\t\t\t\tC Output")
        print(c_result)

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result, matlab_result, rtol=1e-5, atol=1e-8)

        print("MATLAB Input")
        print(v)
        print("MATLAB Output")
        print(matlab_result)
        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")

    def test_unhatSO3(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        v = np.random.rand(3,3)

        # Call the MATLAB function
        matlab_result = matlab_engine.unhat(v)
        matlab_result = np.array(matlab_result)

        npResult = np.zeros((3,1))
        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = lieGroupCall.call_unhatSO3(v,c_result)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing unhatSO3 in C against unhatSO3 in MATLAB")
        print("------------------------------------------------------")
        print("\n\t\t\t\tInput")
        print(v)
        print("\n\t\t\t\tMATLAB Output")
        print(matlab_result)

        print("\n\t\t\t\tC Output")
        print(c_result)

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result, matlab_result, rtol=1e-5, atol=1e-8)

        print("MATLAB Input")
        print(v)
        print("MATLAB Output")
        print(matlab_result)
        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")


    def test_adj(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        v = np.random.rand(4,4)

        # Call the MATLAB function
        matlab_result = matlab_engine.Ad(v)
        matlab_result = np.array(matlab_result)

        npResult = np.zeros((6,6))
        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = lieGroupCall.call_adj(v,c_result)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing adj in C against Ad in MATLAB")
        print("------------------------------------------------------")
        print("\n\t\t\t\tInput")
        print(v)
        print("\n\t\t\t\tMATLAB Output")
        print(matlab_result)

        print("\n\t\t\t\tC Output")
        print(c_result)

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result, matlab_result, rtol=1e-5, atol=1e-8)

        print("MATLAB Input")
        print(v)
        print("MATLAB Output")
        print(matlab_result)
        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")

    def test_adjR6(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        v = np.random.rand(6,1)

        # Call the MATLAB function
        matlab_result = matlab_engine.adj(v)
        matlab_result = np.array(matlab_result)

        npResult = np.zeros((6,6))
        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = lieGroupCall.call_adj_r6(v,c_result)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing adjR6 in C against adj in MATLAB")
        print("------------------------------------------------------")
        print("\n\t\t\t\tInput")
        print(v)
        print("\n\t\t\t\tMATLAB Output")
        print(matlab_result)

        print("\n\t\t\t\tC Output")
        print(c_result)

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result, matlab_result, rtol=1e-5, atol=1e-8)

        print("MATLAB Input")
        print(v)
        print("MATLAB Output")
        print(matlab_result)
        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")

    def test_expm_SO3(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        v = np.random.rand(3,3)

        # Call the MATLAB function
        matlab_result = matlab_engine.expm3(v)
        matlab_result = np.array(matlab_result)

        npResult = np.zeros((3,3))
        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = lieGroupCall.call_expm_SO3(v,c_result)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing expm_SO3 in C against expm3 in MATLAB")
        print("------------------------------------------------------")
        print("\n\t\t\t\tInput")
        print(v)
        print("\n\t\t\t\tMATLAB Output")
        print(matlab_result)

        print("\n\t\t\t\tC Output")
        print(c_result)

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result, matlab_result, rtol=1e-5, atol=1e-8)

        print("MATLAB Input")
        print(v)
        print("MATLAB Output")
        print(matlab_result)
        print("TEST PASSED")

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")

    def test_expm_SE3(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        v = np.random.rand(4,4)

        # Call the MATLAB function
        matlab_result = matlab_engine.expm3(v)
        matlab_result = np.array(matlab_result)

        npResult = np.zeros((6,6))
        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = lieGroupCall.call_expm_SE3(v,c_result)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing expm_SO3 in C against expm3 in MATLAB")
        print("------------------------------------------------------")
        print("\n\t\t\t\tInput")
        print(v)
        print("\n\t\t\t\tMATLAB Output")
        print(matlab_result)

        print("\n\t\t\t\tC Output")
        print(c_result)

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result, matlab_result, rtol=1e-5, atol=1e-8)

        print("MATLAB Input")
        print(v)
        print("MATLAB Output")
        print(matlab_result)
        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")

    def test_all(self, eng):
        self.test_hatR3(eng)
        self.test_hatR6(eng)
        self.test_unhatSO3(eng)
        self.test_adj(eng)
        self.test_adjR6(eng)
        self.test_expm_SO3(eng)


class testMatrix:
    def test_matrix_add_sq(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        randSize =  int(random() * 100)# change 100 to set max size
        m1 = np.random.rand( randSize, randSize)
        m2 = np.random.rand(randSize,randSize)

        #testing agains numpy


        npResult = m1 + m2

        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = matrixCall.call_matrix_add(m1, m2, c_result)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing add in C against add in Numpy")
        print("------------------------------------------------------")

        print("\n\t\t\t\tInput 1")
        print(m1)
        print("\n\t\t\t\tInput 2")
        print(m2)

        print("\n\t\t\t\tMATLAB Output")
        print(npResult)

        print("\n\t\t\t\tC Output")
        print(c_result)

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result, npResult, rtol=1e-5, atol=1e-8)


        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
    def test_matrix_add3(matlab_engine):

        # Define test inputs (ensure these match the values used in the MATLAB test)
        randSize =  int(random() * 100)# change 100 to set max size
        m1 = np.random.rand( randSize, randSize)
        m2 = np.random.rand(randSize,randSize)
        m3 = np.random.rand(randSize,randSize)

        #testing agains numpy


        npResult = m1 + m2 + m3

        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = matrixCall.call_matrix_add3(m1, m2, m3, c_result)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing add3 in C against unhatSO3 in Numpy")
        print("------------------------------------------------------")

        print("\n\t\t\t\tInput 1")
        print(m1)
        print("\n\t\t\t\tInput 2")
        print(m2)
        print("\n\t\t\t\tInput 2")
        print(m3)

        print("\n\t\t\t\tMATLAB Output")
        print(npResult)

        print("\n\t\t\t\tC Output")
        print(c_result)

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result, npResult, rtol=1e-5, atol=1e-8)


        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        
    def test_matrix_sub(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        randSize =  int(random() * 100)# change 100 to set max size
        m1 = np.random.rand( randSize, randSize)
        m2 = np.random.rand(randSize,randSize)


        #testing agains numpy


        npResult = m1 + m2

        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = matrixCall.call_matrix_sub(m1, m2, c_result)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing sub in C against subtracting in Numpy")
        print("------------------------------------------------------")

        print("\n\t\t\t\tInput 1")
        print(m1)
        print("\n\t\t\t\tInput 2")
        print(m2)


        print("\n\t\t\t\tMATLAB Output")
        print(npResult)

        print("\n\t\t\t\tC Output")
        print(c_result)

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result, npResult, rtol=1e-5, atol=1e-8)


        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")

    def test_matrix_mul(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        randSize =  int(random() * 10)

        m1 = np.random.rand( randSize, randSize)
        m2 = np.random.rand(randSize,randSize)

        #testing agains numpy


        npResult = m1 * m2

        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = matrixCall.call_matrix_mult(m1, m2, c_result)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing matrix multiplication in C against matrix multiplication in Numpy")
        print("------------------------------------------------------")

        print("\n\t\t\t\tInput 1")
        print(m1)
        print("\n\t\t\t\tInput 2")
        print(m2)


        print("\n\t\t\t\tMATLAB Output")
        print(npResult)

        print("\n\t\t\t\tC Output")
        print(c_result)

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result, npResult, rtol=1e-5, atol=1e-8)


        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")

    def test_matrix_solve(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        randSize =  int(random() * 10)

        A = np.random.rand(randSize, randSize)
        A = A * (np.eye(randSize)*100)

        B = np.random.rand(randSize, 1)

        #testing agains numpy


        npResult = np.linalg.solve(A,B);

        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = matrixCall.call_matrix_solve(A, B, c_result)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing matrix solve in C against np.linlag.solve in Numpy")
        print("------------------------------------------------------")

        print("\n\t\t\t\tInput 1")
        print(A)
        print("\n\t\t\t\tInput 2")
        print(B)


        print("\n\t\t\t\tMATLAB Output")
        print(npResult)

        print("\n\t\t\t\tC Output")
        print(c_result)

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result, npResult, rtol=1e-5, atol=1e-8)


        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")



def robotCompare(matlab_engine):
    #get and print MATLAB robot

    theta = np.zeros((5,1))
    theta_dot = np.zeros((5,1))
    #Theta_ddot = Shape .* sin(pi/(dt*TimeStep).*t1);
    theta_ddot =  np.zeros((5,1))
    matlab_robot = matlab_engine.defPaperSample_2(theta, theta_dot, theta_ddot)

    print("MATLAB Robot")
    print(matlab_robot)

    theta_m = ctypes.POINTER(matrix)
    theta_m = ctypes.pointer(makeMatrix(theta))

    theta_dot_m = ctypes.POINTER(matrix)
    theta_dot_m = ctypes.pointer(makeMatrix(theta_dot))

    theta_ddot_m = ctypes.POINTER(matrix)
    theta_ddot_m = ctypes.pointer(makeMatrix(theta_ddot))

    #get and print C robot
    c_robot = robotCall.call_defPaperSample2(theta_m, theta_dot_m, theta_ddot_m)



    print("C Robot")
    print(c_robot)



robotCompare(eng)