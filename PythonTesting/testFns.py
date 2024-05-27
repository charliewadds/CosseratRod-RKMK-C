import matlab.engine

from termcolor import colored

print("starting matlab")
eng = matlab.engine.start_matlab()
print("matlab started")
eng.cd(r'/Users/charliewadds/MATLAB/github/MultiBody-Dynamics-LG/Library', nargout=0)
eng.addpath(r'/Users/charliewadds/MATLAB/github/MultiBody-Dynamics-LG/Library', nargout=0)

from callFns import *

class testLieGroup:
    def test_hatR3(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        #v = np.array([[], [2], [3]], dtype=np.float64)
        v = np.random.rand(3,1)
        # Call the MATLAB function
        matlab_result = matlab_engine.hat(v)
        matlab_result = np.array(matlab_result)
        npResult = np.zeros((3,3))
        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = call_hatR3(v,c_result)


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
        print(colored("test passed", 'green'))
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")
    def test_hatR6(matlab_engine):
        # Define test inputs (ensure these match the values used in the MATLAB test)
        v = np.random.rand(6,1)

        # Call the MATLAB function
        matlab_result = matlab_engine.hat(v)
        matlab_result = np.array(matlab_result)

        npResult = np.zeros((4,4))
        c_result = makeMatrix(npResult)
        # Call the C function
        c_result = call_hatR6(v,c_result)

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
        print(colored("test passed", 'green'))
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
        c_result = call_unhatSO3(v,c_result)

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
        print(colored("test passed", 'green'))
        print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||\n\n\n")

testLieGroup.test_hatR3(eng)
testLieGroup.test_hatR6(eng)
testLieGroup.test_unhatSO3(eng)

