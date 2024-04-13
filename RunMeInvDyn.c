//
// Created by Charlie Wadds on 2024-04-09.
//


#include "RobotLib.h"
//
// Created by Charlie Wadds on 2024-03-17.
//

#include "RobotLib.h"
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include <assert.h>


Robot *defPaperSample_2(matrix *theta, matrix *theta_dot, matrix *theta_ddot){
    assert(theta->numCols == 1);//todo add asserts like this to all functions with matrix args
    double linkMass = 12.5;
    double linkLen = 0.5;

    matrix *linkTwist = zeros(6,1);
    linkTwist->data[2][0] = 1;
    linkTwist = matrix_scalar_mul(linkTwist, linkLen);



    matrix *linkCoM = elemDiv(linkTwist, 2);

    matrix *linkInertia = matrix_scalar_mul(eye(3), 1.0/12.0 * linkMass * pow(linkLen, 2));

    matrix *M = eye(6);
    setSection(M, 0, 2, 0, 2, matrix_scalar_mul(eye(3),linkMass));
    setSection(M, 3, 5, 3, 5, linkInertia);

    matrix *Z = zeros(6,1);

    SE3 *Z_SE3 = new_SE3_zeros();

    double L_0 = 0.4;
    matrix *F_0 = zeros(6,1);

    double rho = 75e1;
    double mu = 2e4;
    double r = 0.01;
    double E = 2e8;
    double G = E/(2*(1+0.3));
    double I = M_PI/4*pow(r,2);
    double A = M_PI*pow(r,2);
    int N = 21;

    //diag(2I,I,I)
    matrix *J = zeros(3,3);
    J->data[0][0] = 2*I;
    J->data[1][1] = I;
    J->data[2][2] = I;

    matrix *Kbt = zeros(3,3);
    Kbt->data[0][0] = 2*G*I;
    Kbt->data[1][1] = E*I;
    Kbt->data[2][2] = E*I;

    matrix *Kse = zeros(3,3);
    Kse->data[0][0] = E*A;
    Kse->data[1][1] = G*A;
    Kse->data[2][2] = G*A;

    matrix *Cse = zeros(3,3);
    Cse->data[0][0] = 3*A;
    Cse->data[1][1] = A;
    Cse->data[2][2] = A;
    matrix_scalar_mul(Cse, mu);

    matrix *Cbt = zeros(3,3);
    Cbt->data[0][0] = 2*I;
    Cbt->data[1][1] = I;
    Cbt->data[2][2] = I;
    matrix_scalar_mul(Cbt, mu);

    matrix *K = zeros(6,6);
    setSection(K, 0, 2, 0, 2, Kse);
    setSection(K, 3, 5, 3, 5, Kbt);

    matrix *C = zeros(6,6);
    setSection(C, 0, 2, 0, 2, Cse);
    setSection(C, 3, 5, 3, 5, Cbt);


    matrix *Mf = zeros(6,6);
    setSection(Mf, 0, 2, 0, 2, matrix_scalar_mul(eye(3),rho*A));
    setSection(Mf, 3, 5, 3, 5, matrix_scalar_mul(J,rho));




    //todo find a better way to do this, maybe json or something
    Object *base = malloc(sizeof(union object_u));
    base->type = 0;
    base->object = malloc(sizeof(union object_u));
    base->object->rigid = newRigidBody("base", matrix_scalar_mul(eye(6), DBL_MAX), Z, Z);//todo dbl max to replace inf

    Object *Body_1 =  malloc(sizeof(union object_u));
    Body_1->type = 0;
    Body_1->object = malloc(sizeof(union object_u));
    Body_1->object->rigid = newRigidBody("Body_1", M,  linkTwist, linkCoM);

    Object *Body_2 =  malloc(sizeof(union object_u));
    Body_2->type = 1;
    Body_2->object = malloc(sizeof(union object_u));
    Body_2->object->flex = newFlexBody("Body_2", Mf,  K,C, F_0, N, L_0);

    Object *Body_3 =  malloc(sizeof(union object_u));
    Body_3->type = 0;
    Body_3->object = malloc(sizeof(union object_u));
    Body_3->object->rigid = newRigidBody("Body_3", M,  linkTwist, linkCoM);

    Object *Body_4 =  malloc(sizeof(union object_u));
    Body_4->type = 1;
    Body_4->object = malloc(sizeof(union object_u));
    Body_4->object->flex = newFlexBody("Body_4",Mf,  K,C, F_0, N, L_0);

    Object *Body_5 =  malloc(sizeof(union object_u));
    Body_5->type = 0;
    Body_5->object = malloc(sizeof(union object_u));
    Body_5->object->rigid = newRigidBody("Body_5", elemDiv(M,2), elemDiv(linkTwist,2), elemDiv(linkCoM,2));

    Object *EE =      malloc(sizeof(union object_u));
    EE->type = 0;
    EE->object = malloc(sizeof(union object_u));
    EE->object->rigid = newRigidBody("EE", zeros(6,6),  Z, Z);


    matrix *r6_2 = zeros(6,1);
    r6_2->data[2][0] = 1;

    matrix *r6_3 = zeros(6,1);
    r6_3->data[3][0] = 1;

    matrix *r6_4 = zeros(6,1);
    r6_4->data[4][0] = 1;

    matrix *r6_5 = zeros(6,1);
    r6_5->data[5][0] = 1;

    double *lims = malloc(sizeof(double) * 2);
    lims[0] = -M_PI;
    lims[1] = M_PI;


    double pihalf = M_PI/2;


    //todo this should be in a function like createObject or something
    Object *Joint_1 = malloc(sizeof(Object));
    Joint_1->type = 2;
    Joint_1->object = malloc(sizeof(union object_u));

    Joint_1->object->joint = newRigidJoint("Joint_1", r6_5, theta->data[0][0], theta_dot->data[0][0], theta_ddot->data[0][0], lims, 0, base, Body_1);

    Object *Joint_2 = malloc(sizeof(Object));
    Joint_2->type = 2;
    Joint_2->object = malloc(sizeof(union object_u));
    Joint_2->object->joint = newRigidJoint("Joint_2", r6_3, theta->data[1][0], theta_dot->data[1][0], theta_ddot->data[1][0], lims, pihalf, Body_1, Body_2);

    Object *Joint_3= malloc(sizeof(Object));
    Joint_3->type = 2;
    Joint_3->object = malloc(sizeof(union object_u));
    Joint_3->object->joint = newRigidJoint("Joint_3", r6_4, theta->data[2][0], theta_dot->data[2][0], theta_ddot->data[2][0], lims, 0, Body_2, Body_3);

    Object *Joint_4= malloc(sizeof(Object));
    Joint_4->type = 2;
    Joint_4->object = malloc(sizeof(union object_u));
    Joint_4->object->joint = newRigidJoint("Joint_4", r6_3, theta->data[3][0], theta_dot->data[3][0], theta_ddot->data[3][0], lims, 0, Body_3, Body_4);

    Object *Joint_5= malloc(sizeof(Object));
    Joint_5->type = 2;
    Joint_5->object = malloc(sizeof(union object_u));
    Joint_5->object->joint = newRigidJoint("Joint_5", r6_2, theta->data[4][0], theta_dot->data[4][0], theta_ddot->data[4][0], lims, 0, Body_4, Body_5);

    double zero[2] = {0,0};

    Object *joint_EE = malloc(sizeof(union object_u));
    joint_EE->type = 2;
    joint_EE->object = malloc(sizeof(union object_u));
    joint_EE->object->joint =  newRigidJoint("joint_EE", zeros(6,1), 0, 0, 0, zero, 0, NULL, EE);

    Robot *newRobot = malloc(sizeof(Robot));
    newRobot->name = "RigidRandy";

    //{base, Joint_1, Body_1, Jxoint_2, Body_2, Joint_3, Body_3, Joint_4, Body_4, Joint_5, Body_5, joint_EE, EE};
    Object **robotList = malloc(sizeof(Object) * 13);
    robotList[0] = base;
    robotList[1] = Joint_1;

    robotList[2] = Body_1;
    robotList[3] = Joint_2;
    robotList[4] = Body_2;
    robotList[5] = Joint_3;
    robotList[6] = Body_3;
    robotList[7] = Joint_4;
    robotList[8] = Body_4;
    robotList[9] = Joint_5;
    robotList[10] = Body_5;
    robotList[11] = joint_EE;
    robotList[12] = EE;


    newRobot->objects = robotList;

    newRobot->numObjects = 13;


    return newRobot;
}



void matrixToFile(matrix *m, char *filename){
    FILE *f = fopen(filename, "a");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }

    for (int i = 0; i < m->numRows; i++){
        for (int j = 0; j < m->numCols; j++){
            fprintf(f, "%f, ", m->data[i][j]);
        }
        fprintf(f, "\n");
    }
    fclose(f);
}




int main() {
    matrix *theta = zeros(5, 1);
    matrix *theta_dot = zeros(5, 1);

    double dt = 0.025;
    int timeStep = 100;
    double restTime = 0;

    matrix *t1 = matrix_new(1, timeStep);
    for (int i = 0; i < timeStep; i++) {
        t1->data[0][i] = t1->data[0][i-1] + dt;

    }

    matrix *shape = zeros(5, 1);//todo this is the bymber of joints I think, this should come from either the setup file or a config json or something
    shape->data[0][0] = 0.4;
    shape->data[1][0] = -0.5 * 0.4;
    shape->data[2][0] = 0.5 * 0.4;
    shape->data[3][0] = -0.7 * 0.4;
    shape->data[4][0] = 0.1 * -0.4;

    matrix *theta_ddot = zeros(5, timeStep);


    //todo should be a loop for num bodies
    theta_ddot->data[0] = matrix_scalar_mul(matrix_scalar_mul(t1, sin(PI/(dt*timeStep))),*shape->data[0])->data[0];//todo check shapes
    theta_ddot->data[1] = matrix_scalar_mul(matrix_scalar_mul(t1, sin(PI/(dt*timeStep))),*shape->data[1])->data[0];
    theta_ddot->data[2] = matrix_scalar_mul(matrix_scalar_mul(t1, sin(PI/(dt*timeStep))),*shape->data[2])->data[0];
    theta_ddot->data[3] = matrix_scalar_mul(matrix_scalar_mul(t1, sin(PI/(dt*timeStep))),*shape->data[3])->data[0];
    theta_ddot->data[4] = matrix_scalar_mul(matrix_scalar_mul(t1, sin(PI/(dt*timeStep))),*shape->data[4])->data[0];

    matrix *F_ext = zeros(6, 1);
    matrix *F_0 = zeros(6, 1);
    F_0->data[2][0] = 1;

    Robot *robot = defPaperSample_2(theta, theta_dot, theta_ddot);

    int BC_Start = 2;//todo, this should be automated
    int BC_End = 4;

    matrix *t = zeros(1, timeStep);
    for(int i = 0; i < timeStep; i++){
        t->data[0][i] = i*dt;
    }
    matrix *C = zeros(5, timeStep);//todo 5 should be num bodies
    matrix *T_H = zeros(5, timeStep);//todo 5 should be num bodies
    matrix *Td_H = zeros(5, timeStep);//todo 5 should be num bodies

    matrix *EE_POS = zeros(3, timeStep);


    IDM_MB_RE_OUT *idm = malloc(sizeof(IDM_MB_RE_OUT));
    for(int i = 0; i < timeStep; i++){
        idm = IDM_MB_RE(robot, theta, theta_dot, getSection(theta_ddot, 0, 4, i, i), F_ext, dt, F_0);

        setSection(C, 0, 4, i, i, idm->C);

        flexBody *flex = robot->objects[2*BC_Start ]->object->flex;
        flexBody *flexNew = robot->objects[2*BC_Start]->object->flex;

        //prevGuess is always 1, todo add other cases
        F_0 = getSection(flexNew->f_prev, 0, 5, 0, 0);
        robot = idm->robot_new;

        setSection(T_H, 0, 4, i, i, theta);
        setSection(Td_H, 0, 4, i, i, theta_dot);

        theta = matrix_add(theta, matrix_scalar_mul(getSection(theta_dot, 0, 4, 0, 0), dt));
        theta_dot = matrix_add(theta_dot, matrix_scalar_mul(getSection(getSection(theta_ddot, 0, 4, i, i), 0, 4, 0, 0), dt));

        for(int j = 0; j < 5; j++) {//todo 5 should be num bodies
            if (robot->objects[j]->type == 3) {
                robot->objects[j]->object->joint->position = theta->data[j][0];
                robot->objects[j]->object->joint->velocity = theta_dot->data[j][0];
                robot->objects[j]->object->joint->acceleration = theta_ddot->data[j][i];
            }
        }

        matrixToFile(plotRobotConfig(robot, theta, 100), "RigidRandyPlot.csv");
    }

    free(idm);

}