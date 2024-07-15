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
#include <math.h>
//#include <float.h>
#include <string.h>
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



    double dt = 0.025;
    int timeStep = 100;
    //double restTime = 0;

    matrix *t1 = matrix_new(1, timeStep);
    t1->data[(0 * t1->numCols) + 0] = dt;
    for (int i = 0; i < timeStep; i++) {
        t1->data[(0 * t1->numCols) + i] = t1->data[(0 * t1->numCols) + (i-1)] + dt;

    }

    matrix *shape = zeros(5, 1);
    shape->data[(0 * shape->numCols) + 0] = 0.4;
    shape->data[(1 * shape->numCols) + 0] = -0.5 * 0.4;
    shape->data[(2 * shape->numCols) + 0] = 0.5 * 0.4;
    shape->data[(3 * shape->numCols) + 0] = -0.7 * 0.4;
    shape->data[(4 * shape->numCols) + 0] = 0.1 * -0.4;

    matrix *theta_ddot = zeros(5, 1);
    matrix *theta = zeros(5, 1);
    matrix *theta_dot = zeros(5, 1);
    matrix *tempTStep = matrix_new(1, timeStep);

    matrix *F_ext = zeros(6, 1);
    matrix *F_0 = zeros(6, 1);
    F_0->data[(0 * F_0->numCols)] = 0;
    F_0->data[(1 * F_0->numCols)] = 0;
    F_0->data[(2 * F_0->numCols)] = 1;
    F_0->data[(3 * F_0->numCols)] = 0;
    F_0->data[(4 * F_0->numCols)] = 0;
    F_0->data[(5 * F_0->numCols)] = 0;

    //matrix *temp1xRowsM1 = matrix_new(5, timeStep);
    matrix *tempBodiesx1 = matrix_new(5, 1);

    Robot *robot = defPaperSample_1(theta, theta_dot, getSection(theta_ddot, 0, theta_ddot->numRows-1, 0, 0, tempBodiesx1));//todo check -1

    int BC_Start = 2;//todo, this should be automated
    //int BC_End = 4;

    matrix *t = zeros(1, timeStep);
    for(int i = 0; i < timeStep; i++){
        t->data[(0 * t->numCols) + i] = i*dt;
    }
    matrix *C = zeros(5, timeStep);//todo 5 should be num bodies
    matrix *T_H = zeros(5, timeStep);//todo 5 should be num bodies
    matrix *Tdd_H = zeros(5, timeStep);//todo 5 should be num bodies

    matrix *angles = zeros(((robot->numObjects-1)/2)-1,timeStep);
    FDM_MB_RE_OUT *fdm = malloc(sizeof(FDM_MB_RE_OUT));
    matrix *tempLinkx1 = matrix_new(5,1);
    matrix *InitGuess = zeros(5,1);


    matrix *C_des = matrix_new(5,100);
    matrixFromFile("Control_good.csv", C_des);
    matrix *C_des_1 = zeros(5,1);

    matrix *temp5x1 = zeros(5,1);
    matrix *temp5xn = zeros(5,timeStep);
    matrix *tempF = zeros(6,1);
    matrix* negative = ones(1,5);
    matrix_scalar_mul(negative, 0, negative);


    for(int i = 0; i < timeStep; i++){
#if VERBOSE > 0
        printf("\nTime Step: %d\n", i);

        stepStart = clock();
#endif
        #if LOG_F_FLEX == 1
        matrixToFile(negative, "C_inv.csv");
        matrixToFile(negative, "InitGuess.csv");
        #endif
        setSection(C_des_1, 0, C_des_1->numRows-1, 0, 0, getSection(C_des, 0, C_des->numRows-1, i, i, tempLinkx1));



        fdm = FDM_MB_RE(robot, theta, theta_dot, theta_ddot, F_ext, dt, C_des_1 ,F_0, InitGuess);//todo do I need JointAcc in funciton?


        matrix *tempT6 = matrix_new(1, 6);
        matrix *tempf = matrix_new(7, 6);
        matrix *tempT = matrix_new(1, 5);
        matrixToFile(matrix_transpose(fdm->C, tempT), "C.csv");
        matrixToFile(matrix_transpose(fdm->JointAcc, tempT), "jointAcc.csv");
        matrixToFile(matrix_transpose(theta_dot, tempT), "theta_dot.csv");
        matrixToFile(matrix_transpose(fdm->F, tempf), "F.csv");





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
        //TODO this diverges after the second step

        copyMatrix(fdm->JointAcc, theta_ddot);

        matrix_scalar_mul(theta_ddot, dt, temp5x1);
        matrix_add(temp5x1, theta_dot, theta_dot);

        matrix_scalar_mul(theta_dot, dt, temp5x1);
        matrix_add(temp5x1, theta, theta);

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
        int curr = 0;
        for(int j = 0; j < robot->numObjects-1; j++){
            if(robot->objects[j]->type == 2){
                curr ++;
                #if VERBOSE > 0
                printf("Joint %d: %f\n", j, robot->objects[j]->object->joint->position);
                #endif
                angles->data[(curr * angles->numCols) + i] = robot->objects[j]->object->joint->position;

            }
        }

        assert(isnan(robot->objects[1]->object->joint->velocity)==0);

#if VERBOSE > 0
        printf("step took: %f Seconds\n", ((double) (clock() - stepStart)) / CLOCKS_PER_SEC);
#endif
        matrixToFile(plotRobotConfig(robot, theta, 2), "RigidRandyPlot_1.csv");
        //saveTimeCSV(i, ((double) (clock() - stepStart)) / CLOCKS_PER_SEC, "time.csv");
    }
    printf("DONE");
//    matrixToFile(angles, "RigidRandyAngles.csv");
//    robotToFile(robot, "testRobotOut.json");
    matrixToFile(angles, "RigidRandyAngles_fwd.csv");
    matrix_free(tempBodiesx1);
    matrix_free(tempLinkx1);
    matrix_free(shape);
    matrix_free(C);
    matrix_free(T_H);
    matrix_free(Tdd_H);


    matrix_free(fdm->C);
    matrix_free(fdm->F);
    matrix_free(fdm->JointAcc);
    free(fdm);

    matrix_free(F_ext);
    matrix_free(F_0);
    robotFree(robot);
    matrix_free(theta);
    matrix_free(theta_dot);
    matrix_free(theta_ddot);

    end = clock();
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("Time: %f\n", cpu_time_used);
}