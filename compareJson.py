import json
import math

import numpy as np
import pytest
from pytest import approx


def compareVals(dict1, dict2, name):
    if(type(dict1.get(name)) == float  or type(dict1.get(name)) == int):
        assert dict1.get(name) == approx(dict2.get(name), abs=1e-15)
    elif(type(dict1.get(name)) == list):


        if(type(dict1.get(name)[0]) == list):
            print(name)
            for i in range(0, len(dict1.get(name))):
                for j in range(0, len(dict1.get(name)[i])):
                    if(dict2.get(name)[i][j] != None and dict1.get(name)[i][j] != None):
                        assert dict1.get(name)[i][j] == approx(dict2.get(name)[i][j], abs=1e-15), "i: " + str(i) + " j: " + str(j) +"|  | "+ str(dict1.get(name)[i][j]) + " != " + str(dict2.get(name)[i][j])
        else:
            if(dict2.get(name) != None and dict1.get(name) != None):
                assert dict1.get(name) == approx(dict2.get(name), abs=1e-15)
    elif(type(dict1.get(name)) == str):
        assert dict1.get(name).lower() == dict2.get(name).lower()
    elif(type(dict1.get(name)) == dict):
        for i in dict2.get(name).keys():
            compareVals(dict1.get(name), dict2.get(name), i)
        #compareVals(dict1.get(name), dict2.get(name), name)
    else:
        print("Unknown type for " + name)
        assert False

def test_compareRobot(matlabRobot, robot2):
    # Open the JSON files


    for i in range(0, len(matlabRobot)):
        print("testing " + matlabRobot[i].get("Name") + " against " + robot2[i].get("Name"))

        print("lenKeys: " + str(len(robot2[i].keys())))
        for j in range(0, len(robot2[i].keys())):
            print("i; " + str(i))

            print("\ttesting " + list(robot2[i].keys())[j] + " against " + list(matlabRobot[i].keys())[j])
            compareVals(matlabRobot[i], robot2[i], list(robot2[i].keys())[j])


def test_testList():
    with open('testData/matlabKnownGood/update.json') as f:
        matlabRobot = json.load(f)
    with open('cmake-build-debug/testRobotOut.json') as f:
        robot2 = json.load(f)

    for i in range(0, len(matlabRobot)):
        #print("testing " + matlabRobot[i].get("Name") + " against " + robot2[i].get("Name"))
        test_compareRobot(matlabRobot.get(str(i)), robot2.get(str(i)))
test_testList()
