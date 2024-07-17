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





#define TEMP_NUMBODY 5
int main() {

    clock_t start, end;
    double cpu_time_used;

    start = clock();

    matrix *theta = zeros(TEMP_NUMBODY , 1);
    matrix *theta_dot = zeros(TEMP_NUMBODY , 1);

    double dt = 0.025;
    int timeStep = 100;
    //double restTime = 0;

    matrix *t1 = matrix_new(1, timeStep);
    t1->data[(0 * t1->numCols) + 0] = dt;
    for (int i = 1; i < timeStep; i++) {
        t1->data[(0 * t1->numCols) + i] = t1->data[(0 * t1->numCols) + (i-1)] + dt;

    }

    matrix *shape = zeros(TEMP_NUMBODY, 1);
    shape->data[(0 * shape->numCols) + 0] = 0.4;
    shape->data[(1 * shape->numCols) + 0] = -0.5 * 0.4;
    shape->data[(2 * shape->numCols) + 0] = 0.5 * 0.4;
    shape->data[(3 * shape->numCols) + 0] = -0.7 * 0.4;
    shape->data[(4 * shape->numCols) + 0] = 0.1 * -0.4;

    matrix *theta_ddot = zeros(TEMP_NUMBODY, timeStep);

    matrix *tempTStep = matrix_new(1, timeStep);

    for(int i = 0; i < TEMP_NUMBODY; i++){

        zeroMatrix(tempTStep);
        matrix_scalar_mul(matrix_sin(matrix_scalar_mul(t1, PI/(dt*timeStep), tempTStep)),shape->data[(i * shape->numCols)], tempTStep);
        //memcpy(theta_ddot->data[i],tempTStep->data[0],  sizeof * theta_ddot->data[i] * timeStep);
        setSection(theta_ddot, i, i, 0, timeStep-1, tempTStep);

    }

    matrix *F_ext = zeros(6, 1);
    matrix *F_0 = zeros(6, 1);
    F_0->data[(0 * F_0->numCols)] = 0;
    F_0->data[(1 * F_0->numCols)] = 0;
    F_0->data[(2 * F_0->numCols)] = 1;
    F_0->data[(3 * F_0->numCols)] = 0;
    F_0->data[(4 * F_0->numCols)] = 0;
    F_0->data[(5 * F_0->numCols)] = 0;



    //matrix *temp1xRowsM1 = matrix_new(5, timeStep);
    matrix *tempBodiesx1 = matrix_new(TEMP_NUMBODY, 1);

    Robot *robot = defPaperSample_2(theta, theta_dot, getSection(theta_ddot, 0, theta_ddot->numRows-1, 0, 0, tempBodiesx1));//todo check -1

    int BC_Start = 2;//todo, this should be automated
    //int BC_End = 4;

    matrix *t = zeros(1, timeStep);
    for(int i = 0; i < timeStep; i++){
        t->data[(0 * t->numCols) + i] = i*dt;
    }
    matrix *C = zeros(robot->numBody, timeStep);
    matrix *T_H = zeros(robot->numBody, timeStep);
    matrix *Td_H = zeros(robot->numBody, timeStep);

    //matrix *EE_POS = zeros(3, timeStep);

    matrix *angles = zeros(((robot->numObjects-1)/2)-1,timeStep);
    IDM_MB_RE_OUT *idm = malloc(sizeof(IDM_MB_RE_OUT));
    matrix *tempLinkx1 = matrix_new(robot->numBody,1);
    for(int i = 0; i < timeStep; i++){

        matrix *f = matrix_new(6,1);
        getSection(robot->objects[(2*robot->BC_Start)]->object->flex->f_prev, 0, robot->objects[(2*robot->BC_Start)]->object->flex->f_prev->numRows - 1, 0, 0, f);//todo

        idm = IDM_MB_RE(robot, theta, theta_dot, getSection(theta_ddot, 0, theta_ddot->numRows-1, i, i, tempLinkx1), F_ext, dt, F_0);

        matrixToFile(f, "f.csv");
        matrix *tempf = matrix_new(7, 6);


        //printf("%f", robot->objects[11]->object->joint->limits[0]);
        setSection(C, 0, C->numRows-1, i, i, idm->C);


        //flexBody *flex = robot->objects[2*BC_Start ]->object->flex;
        //flexBody *flexNew = robot->objects[2*BC_Start]->object->flex;

        //prevGuess is always 1, todo add other cases
        getSection(f, 0, 5, 0, 0, F_0);
        //robot = idm->robot_new;


        setSection(T_H, 0, T_H->numRows-1, i, i, theta);
        setSection(Td_H, 0, Td_H->numRows-1, i, i, theta_dot);


        theta = matrix_add(theta, matrix_scalar_mul(getSection(theta_dot, 0, theta_dot->numRows-1, 0, 0, tempLinkx1), dt, tempLinkx1), theta);
        theta_dot = matrix_add(theta_dot, matrix_scalar_mul(getSection(theta_ddot, 0, theta_ddot->numRows-1, i, i, tempLinkx1), dt, tempLinkx1), theta_dot);//todo this feels wrong
        int currJointIndex = 0;
        for(int j = 1; j < 11; j+= 2 ) {//todo j should start at firstjoint an
            if (robot->objects[j]->type == 2) {
                robot->objects[j]->object->joint->position = theta->data[(currJointIndex * theta->numCols)];
                robot->objects[j]->object->joint->velocity = theta_dot->data[(currJointIndex * theta_dot->numCols)];
                robot->objects[j]->object->joint->acceleration = theta_ddot->data[(currJointIndex * theta_ddot->numCols) + i];
                currJointIndex++;
            }
        }

        int curr = 0;
        for(int j = 0; j < robot->numObjects; j++){
            if(robot->objects[j]->type == 2){

                //printf("Joint %d: %f\n", j, robot->objects[j]->object->joint->position);
                angles->data[(curr * angles->numCols) + (i)] = robot->objects[j]->object->joint->position;
                curr ++;
            }
        }

        matrixToFile(plotRobotConfig(robot, theta, 2), "RigidRandyPlot.csv");
    }
    printf("DONE");
//    matrixToFile(angles, "RigidRandyAngles.csv");
//    robotToFile(robot, "testRobotOut.json");
    //matrixToFile(C, "ControlSim2.csv");
#if INV_SAVE == 1
    matrixToFile(angles, "RigidRandyAngles.csv");
    //robotToFile(robot, "testRobotOut.json");
    matrixToFile(C, "Control_good.csv");
#endif
    matrix_free(tempBodiesx1);
    matrix_free(tempLinkx1);
    matrix_free(shape);
    matrix_free(C);
    matrix_free(T_H);
    matrix_free(Td_H);


    matrix_free(idm->C);
    matrix_free(idm->F);
    matrix_free(idm->v);
    free(idm);

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