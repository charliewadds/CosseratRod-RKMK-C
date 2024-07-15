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





#define TEMP_NUMBODY 3
int main() {

    clock_t start, end;
    double cpu_time_used;

    start = clock();

    matrix *theta = zeros(TEMP_NUMBODY , 1);
    matrix *theta_dot = zeros(TEMP_NUMBODY , 1);

//    double dt = 0.025;
//    int timeStep = 10;
//    double restTime = 20;
//
//    matrix *t1 = matrix_new(1, timeStep);
//    t1->data[(0 * t1->numCols) + 0] = dt;
//    for (int i = 1; i < timeStep; i++) {
//        t1->data[(0 * t1->numCols) + i] = t1->data[(0 * t1->numCols) + (i-1)] + dt;
//
//    }
//
//    matrix *shape = zeros(TEMP_NUMBODY, 1);
//    shape->data[(0 * shape->numCols) + 0] = 0.4;
//    shape->data[(1 * shape->numCols) + 0] = -0.5 * 0.4;
//    shape->data[(2 * shape->numCols) + 0] = 0.5 * 0.4;
//    shape->data[(3 * shape->numCols) + 0] = -0.7 * 0.4;
//    shape->data[(4 * shape->numCols) + 0] = 0.1 * -0.4;
//
//    matrix *theta_ddot = zeros(TEMP_NUMBODY, timeStep)
//    matrix *tempTStep = matrix_new(1, timeStep);
//
//    for(int i = 0; i < TEMP_NUMBODY; i++){
//
//        zeroMatrix(tempTStep);
//        matrix_scalar_mul(matrix_sin(matrix_scalar_mul(t1, PI/(dt*timeStep), tempTStep)),shape->data[(i * shape->numCols)], tempTStep);
//        //memcpy(theta_ddot->data[i],tempTStep->data[0],  sizeof * theta_ddot->data[i] * timeStep);
//        setSection(theta_ddot, i, i, 0, timeStep-1, tempTStep);
//
//    }

    /*
 * %Shape       = [6;-4;-6];                                %[rad/s2]   Actuated Joint Acceleration Shape
%Theta_ddot1 = Shape .* ones(1,TimeStep);                %[rad/s2]   Joint Trajectory 1
%Theta_ddot2 = -Shape.* ones(1,TimeStep);                %[rad/s2]   Joint Trajectory 2
%Theta_ddot3 = 0 .* Shape .* ones(1,TimeStep*RestTime);  %[rad/s2]   Joint Trajectory 3
%Theta_ddot  = [Theta_ddot1, Theta_ddot2, Theta_ddot3];  %[rad/s2]   Joint Trajectory Combined

 */

    //________________________sample 2________________________________________________

    double dt = 0.025;
    int timeStep = 100;
    int restTime = 1;//1 for no rest time
    matrix *shape = matrix_new(TEMP_NUMBODY, 1);
    shape->data[(0 * shape->numCols)] = 6;
    shape->data[(1 * shape->numCols)] = -4;
    shape->data[(2 * shape->numCols)] = -6;

    matrix *theta_ddot = matrix_new(TEMP_NUMBODY, timeStep);

    matrix *tempTStep = matrix_new(1, timeStep);
    matrix *tempTStep1 = ones(1, 10);
    matrix *tempTStep2 = ones(1, 10);

    matrix *add = ones(1, 10);
    for(int i = 0; i < TEMP_NUMBODY; i++){

        matrix_scalar_mul(tempTStep1,shape->data[(i * shape->numCols)], tempTStep1);
        matrix_scalar_mul(tempTStep2,shape->data[(i * shape->numCols)] * -1, tempTStep2);

        setSection(theta_ddot1, i, i, 0, 9, tempTStep1);
        setSection(theta_ddot2, i, i, 0, 9, tempTStep2);
        setSection(theta_ddot, i, i, 20, theta_ddot->numCols-1, theta_ddot3);

        zeroMatrix(tempTStep1);
        zeroMatrix(tempTStep2);
        matrix_add(tempTStep1, add, tempTStep1);
        matrix_add(tempTStep2, add, tempTStep2);
    }
    setSection(theta_ddot, 0, 2, 0, 9, theta_ddot1);
    setSection(theta_ddot, 0, 2, 10, 19, theta_ddot2);

//_____________________________________________________________________________________________________________________

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

    Robot *robot = defPaperSample_1(theta, theta_dot, getSection(theta_ddot, 0, theta_ddot->numRows-1, 0, 0, tempBodiesx1));//todo check -1

    int BC_Start = 2;//todo, this should be automated
    //int BC_End = 4;

    matrix *t = zeros(1, theta_ddot->numCols);
    for(int i = 0; i < theta_ddot->numCols; i++){
        t->data[(0 * t->numCols) + i] = i*dt;
    }
    matrix *C = zeros(robot->numBody, theta_ddot->numCols);//todo 5 should be num bodies
    matrix *T_H = zeros(robot->numBody, theta_ddot->numCols);
    matrix *Td_H = zeros(robot->numBody, theta_ddot->numCols);

    //matrix *EE_POS = zeros(3, timeStep);

    matrix *angles = zeros(((robot->numObjects-1)/2)-1,timeStep);
    IDM_MB_RE_OUT *idm = malloc(sizeof(IDM_MB_RE_OUT));
    matrix *tempLinkx1 = matrix_new(robot->numBody,1);

    for(int i = 0; i < theta_ddot->numCols; i++){

        printf("Time Step: %d\n", i);
        matrix *f = matrix_new(6,1);
        getSection(robot->objects[(2*robot->BC_Start)]->object->flex->f_prev, 0, robot->objects[(2*robot->BC_Start)]->object->flex->f_prev->numRows - 1, 0, 0, f);//todo

        idm = IDM_MB_RE(robot, theta, theta_dot, getSection(theta_ddot, 0, theta_ddot->numRows-1, i, i, tempLinkx1), F_ext, dt, F_0);


        matrix *tempf = matrix_new(idm->F->numCols, idm->F->numRows);
        matrixToFile(matrix_transpose(idm->F, tempf), "Finv_main.csv");

        //printf("%f", robot->objects[11]->object->joint->limits[0]);
        setSection(C, 0, C->numRows-1, i, i, idm->C);

        matrixToFile(idm->C, "C_inv.csv");

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

        matrixToFile(plotRobotConfig(robot, theta, 2), "sample1Plot.csv");
    }
    printf("DONE");


#if INV_SAVE == 1
    matrixToFile(angles, "sample1Angles.csv");
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