//
// Created by Charlie Wadds on 2024-02-28.
//




#include "RobotLib.h"

#include <stdio.h>
int main(void){

    matrix *test = zeros(4,4);
    test->data[0][0] = 3;
    test->data[0][1] = 2;
    test->data[0][2] = 5;
    test->data[0][3] = 7;

    test->data[1][0] = 2;
    test->data[1][1] = 7;
    test->data[1][2] = 4;
    test->data[1][3] = 7;

    test->data[2][0] = 4;
    test->data[2][1] = 1;
    test->data[2][2] = 4;
    test->data[2][3] = 8;

    test->data[3][0] = 13;
    test->data[3][1] = 14;
    test->data[3][2] = 15;
    test->data[3][3] = 16;


    //for this example the rod starts at the origin of the global frame
    rigidBody *rod = malloc(sizeof(rigidBody));
    rod->name = "rod";
    rod->mass = 1;

    matrix *P = matrix_new(3,1);
    P->data[0][0] = 0.5;
    P->data[1][0] = 0;
    P->data[2][0] = 0;
    SE3 *pose = new_SE3_T(test);
    P->data[0][0] = 1;
    matrix *CoM = zeros(6,1);

    rod->Transform = pose;
    rod->CoM = CoM;

    rigidJoint *joint = malloc(sizeof(rigidJoint));
    joint->name = "joint";
    joint->twistR6 = zeros(6,1);
    joint->twistR6->data[0][5] = 1;
    joint->position = 0;
    joint->velocity = 0;
    joint->acceleration = 0;
    joint->limits = malloc(sizeof(float) * 2);
    joint->limits[0] = -3.1416;
    joint->limits[1] = 3.1416;
    joint->homepos = 0;//todo this is weird
    joint->parent = NULL;
    joint->child = rod;


    SE3 *g_old = new_SE3_T(test);
    printMatrix(test);
    printf("the det of test is: %f\n",Det(test));
    SE3 *g_oldToCur = new_SE3_T(test);

    printf(jointToJson(joint));

    matrix *eta_old = zeros(6,1);
    matrix *d_eta_old = zeros(6,1);
    rigidKin *kin = actuateRigidJoint(g_old, g_oldToCur, joint, eta_old, d_eta_old);
    joint->position = 3;
    printf("\nrigidKin output: \n");
    printf("g_cur: \n");
    printMatrix(kin->g_cur->T);
    printf("g_act_wrt_prev: \n");
    printMatrix(kin->g_act_wrt_prev->T);
    printf("eta: \n");
    printMatrix(kin->eta);
    printf("d_eta: \n");
    printMatrix(kin->d_eta);


    return 0;




}

