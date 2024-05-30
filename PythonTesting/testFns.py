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





class testRobot:
    def test_coss_ode(matlab_engine):


        #testing agains numpy
        eta = np.random.rand(6, 1)
        f = np.random.rand(6, 1)
        eta_h = np.random.rand(6, 1)
        f_h = np.random.rand(6, 1)
        f_sh = np.random.rand(6, 1)
        K = np.random.rand(6, 6)
        C = np.random.rand(6, 6)
        M = np.random.rand(6, 6)
        c0 = 60.0
        f_0 = np.random.rand(6, 1)
        Fd_ext = np.random.rand(6, 1)



        matlabOut = matlab_engine.COSS_ODE_TEST(eta, f, eta_h, f_h, f_sh, K, C, M, c0, f_0, Fd_ext)



        # Call the C function
        c_result, c_result1 = robotCall.call_ode(eta, f, eta_h, f_h, f_sh, K, C, M, c0, f_0, Fd_ext)

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing COSS_ODE f_s in C against Coss_ODE f_s in MATLAB")
        print("------------------------------------------------------")

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result[0:2,0], matlabOut[0], rtol=1e-5, atol=1e-8)


        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")

        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
        print("testing COSS_ODE eta_s in C against Coss_ODE eta_s in MATLAB")
        print("------------------------------------------------------")

        # Compare results (you might need to adjust the tolerance based on the expected numerical accuracy)
        np.testing.assert_allclose(c_result_eta_s, matlabOut[1], rtol=1e-5, atol=1e-8)


        print("TEST PASSED")
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")

#testLieGroup.test_all(testLieGroup,eng);#todo I havent used python in a while, this is not the right way to do this
# testMatrix.test_matrix_mul(eng)
#testMatrix.test_matrix_solve(eng)
testRobot.test_coss_ode(eng)