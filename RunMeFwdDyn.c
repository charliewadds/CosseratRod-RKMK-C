//
// Created by Charlie Wadds on 2024-04-09.
//

#include <time.h>
#include "RobotLib.h"
//
// Created by Charlie Wadds on 2024-03-17.
//

#include "RobotLib.h"
#include <stdio.h>

//#include <float.h>

#include "Matrices.h"
#include <assert.h>


void saveTimeCSV(int step, double time, char *filename){
    FILE *f = fopen(filename, "a");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }

    fprintf(f, "%d, %f\n", step, time);

    fclose(f);
}

int main() {

    clock_t start, end;
    double cpu_time_used;

    start = clock();
    clock_t stepStart = 0;

    matrix *tempBodiesx1 = matrix_new(5, 1);
#ifdef SAMPLE2
    double dt = 0.025;
    int timeStep = 100;
    int totTime = 100;
    //double restTime = 0;

    // matrix *t1 = matrix_new(1, timeStep);
    // t1->data[(0 * t1->numCols) + 0] = dt;
    // for (int i = 0; i < timeStep; i++) {
    //     t1->data[(0 * t1->numCols) + i] = t1->data[(0 * t1->numCols) + (i-1)] + dt;
    //
    // }


    matrix *theta_ddot = zeros(5, 1);
    matrix *theta = zeros(5, 1);
    matrix *theta_dot = zeros(5, 1);

    matrix *InitGuess = zeros(5,1);


    Robot *robot = defPaperSample_2(theta, theta_dot, getSection(theta_ddot, 0, theta_ddot->numRows-1, 0, 0, tempBodiesx1));//todo check -1

    matrix *C_des = matrix_new(robot->numBody,totTime);
    matrixFromFile("ControlSim2.csv", C_des);

#endif

#ifdef SAMPLE1
    double dt = 0.025;
    int timeStep = 10;
    int restTime = 5;
    int totTime = 70;

    matrix *theta = zeros(NUMBODIES, 1);
    matrix *theta_dot = zeros(NUMBODIES, 1);
    matrix *theta_ddot = zeros(NUMBODIES, 1);
    matrix *InitGuess = zeros(NUMBODIES, 1);
    InitGuess->data[0] = 6;
    InitGuess->data[1] = -4;
    InitGuess->data[2] = -6;


    Robot *robot = defPaperSample_1(theta, theta_dot, theta_ddot);
    matrix *C_des = matrix_new(robot->numBody,totTime);
    matrixFromFile("../testData/C_DES_MATLAB/PaperSample1_mat.csv", C_des);
#endif

    int BC_Start = robot->BC_Start;//todo, this should be automated
    //int BC_End = 4;

    matrix *t = zeros(1, totTime);
    for(int i = 0; i < timeStep; i++){
        t->data[(0 * t->numCols) + i] = i*dt;
    }
    matrix *C = zeros(5, totTime);//todo 5 should be num bodies
    matrix *T_H = zeros(5, totTime);//todo 5 should be num bodies
    matrix *Tdd_H = zeros(5, totTime);//todo 5 should be num bodies

    matrix *angles = zeros(((robot->numObjects-1)/2)-1,totTime);
    FDM_MB_RE_OUT *fdm = malloc(sizeof(FDM_MB_RE_OUT));
    matrix *tempLinkx1 = matrix_new(robot->numBody,1);


    matrix *F_ext = zeros(6, 1);
    matrix *F_0 = zeros(6, 1);
    F_0->data[(0 * F_0->numCols)] = 0;
    F_0->data[(1 * F_0->numCols)] = 0;
    F_0->data[(2 * F_0->numCols)] = 1;
    F_0->data[(3 * F_0->numCols)] = 0;
    F_0->data[(4 * F_0->numCols)] = 0;
    F_0->data[(5 * F_0->numCols)] = 0;


    matrix *C_des_1 = zeros(C_des->numRows,1);

    matrix *tempNumbodx1 = zeros(robot->numBody,1);
    //matrix *temp5xn = zeros(5,totTime);
    matrix *tempF = zeros(6,1);

    matrix *tempT = matrix_new(1, robot->numBody);
    for(int i = 0; i < totTime; i++){
        #if VERBOSE > 0
        printf("\nTime Step: %d\n", i);

        stepStart = clock();
        #endif
        #if LOG_F_FLEX == 1
        matrixToFile(negative, "C_inv.csv");
        matrixToFile(negative, "InitGuess.csv");
        #endif
        setSection(C_des_1, 0, C_des_1->numRows-1, 0, 0, getSection(C_des, 0, C_des->numRows-1, i, i, tempLinkx1));


        fdm = FDM_MB_RE(robot, theta, theta_dot, theta_ddot, F_ext, dt, C_des_1 ,F_0, InitGuess);//todo run in parallel with 1/2 dt for RK2 or RK4 (or initguess for next ts)


//        matrix *tempT6 = matrix_new(1, 6);
//        matrix *tempf = matrix_new(7, 6);

//        matrixToFile(matrix_transpose(fdm->C, tempT), "C.csv");
        matrixToFile(matrix_transpose(fdm->JointAcc, tempT), "jointAcc.csv");
//        matrixToFile(matrix_transpose(theta, tempT), "theta.csv");
//        matrixToFile(fdm->F, "F.csv");





        assert(isnan(robot->objects[1]->object->joint->velocity)==0);
        setSection(T_H, 0, T_H->numRows-1, i, i, theta);
        setSection(Tdd_H, 0, Tdd_H->numRows-1, i, i, theta_ddot);
        setSection(C, 0, C->numRows-1, i, i, fdm->C);
        copyMatrix(fdm->JointAcc, InitGuess);
#if VERBOSE > 0

        printf("Joint Acc in main: \n");
        printMatrix(fdm->JointAcc);
        printf("-----------------------\n");
#endif
        getSection(robot->objects[2*BC_Start]->object->flex->f_prev, 0, robot->objects[2*BC_Start]->object->flex->f_prev->numRows-1, 0,0, tempF);
        copyMatrix(tempF, F_0);


        copyMatrix(fdm->JointAcc, theta_ddot);

        matrix_scalar_mul(theta_ddot, dt, tempNumbodx1);
        matrix_add(tempNumbodx1, theta_dot, theta_dot);

        matrix_scalar_mul(theta_dot, dt, tempNumbodx1);
        matrix_add(tempNumbodx1, theta, theta);

        int currJointIndex = 0;
        assert(hasNan(theta) == 0);
        assert(hasNan(theta_dot) == 0);
        assert(hasNan(theta_ddot) == 0);
        assert(isnan(robot->objects[1]->object->joint->velocity)==0);

        int num = 0;
        for(int j = 0; j < (robot->numObjects/2)-1; j++){
            if(robot->objects[(2*j)+1]->type == 2){
                robot->objects[(2*j)+1]->object->joint->position = theta->data[num];
                robot->objects[(2*j)+1]->object->joint->velocity = theta_dot->data[num];
                robot->objects[(2*j)+1]->object->joint->acceleration = theta_ddot->data[num];
                num++;
            }
        }


        assert(isnan(robot->objects[1]->object->joint->velocity)==0);

#if VERBOSE > 0
        printf("step took: %f Seconds\n", ((double) (clock() - stepStart)) / CLOCKS_PER_SEC);
#endif
#if PLOT_OUT == 1
        matrix *posOut = plotRobotConfig(robot, theta, 1);
        matrixToFile(posOut, "ForwardDynPlot.csv");
        matrix_free(posOut);
#endif
        //saveTimeCSV(i, ((double) (clock() - stepStart)) / CLOCKS_PER_SEC, "time.csv");
    }
    printf("DONE");

#if PLOT_OUT == 1
    matrixToFile(angles, "ForwardDynAngles.csv");
#endif
    matrix_free(tempBodiesx1);
    matrix_free(tempLinkx1);
    matrix_free(C);
    matrix_free(T_H);
    matrix_free(Tdd_H);


    matrix_free(fdm->C);
    matrix_free(fdm->F);
    matrix_free(fdm->JointAcc);
    free(fdm);

    matrix_free(tempT);
    matrix_free(F_ext);
    matrix_free(F_0);
    robotFree(robot);
    matrix_free(theta);
    matrix_free(theta_dot);
    matrix_free(theta_ddot);
    matrix_free(C_des);
    matrix_free(angles);
    matrix_free(InitGuess);

    end = clock();
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("Time: %f\n", cpu_time_used);
}