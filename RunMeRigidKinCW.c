//
// Created by Charlie Wadds on 2024-03-17.
//

#include "RobotLib.h"
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include <assert.h>


Robot *defRigidKin(matrix *theta, matrix *theta_dot, matrix *theta_ddot){
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

    //todo find a better way to do this, maybe json or something
    union object_u *base =    malloc(sizeof(union object_u));
    base->rigid = newRigidBody("base", matrix_scalar_mul(eye(6), DBL_MAX), Z, Z);//todo dbl max to replace inf
    union object_u *Body_1 =  malloc(sizeof(union object_u));
    Body_1->rigid = newRigidBody("Body_1", M,  linkTwist, linkCoM);
    union object_u *Body_2 =  malloc(sizeof(union object_u));
    Body_2->rigid = newRigidBody("Body_2", M,  linkTwist, linkCoM);
    union object_u *Body_3 =  malloc(sizeof(union object_u));
    Body_3->rigid = newRigidBody("Body_3", M,  linkTwist, linkCoM);
    union object_u *Body_4 =  malloc(sizeof(union object_u));
    Body_4->rigid = newRigidBody("Body_4", M,  linkTwist, linkCoM);
    union object_u *Body_5 =  malloc(sizeof(union object_u));
    Body_5->rigid = newRigidBody("Body_5", elemDiv(M,2), elemDiv(linkTwist,2), elemDiv(linkCoM,2));
    union object_u *EE =      malloc(sizeof(union object_u));
    EE->rigid = newRigidBody("EE", zeros(6,6),  Z, Z);


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
    union object_u *Joint_1 = malloc(sizeof(union object_u));
    Joint_1->joint = newRigidJoint("Joint_1", r6_5, theta->data[0][0], theta_dot->data[0][0], theta_ddot->data[0][0], lims, 0, base->rigid, Body_1->rigid);
    union object_u *Joint_2 = malloc(sizeof(union object_u));
    Joint_2->joint = newRigidJoint("Joint_2", r6_3, theta->data[1][0], theta_dot->data[1][0], theta_ddot->data[1][0], lims, pihalf, Body_1->rigid, Body_2->rigid);
    union object_u *Joint_3= malloc(sizeof(union object_u));
    Joint_3->joint = newRigidJoint("Joint_3", r6_4, theta->data[2][0], theta_dot->data[2][0], theta_ddot->data[2][0], lims, 0, Body_2->rigid, Body_3->rigid);
    union object_u *Joint_4= malloc(sizeof(union object_u));
    Joint_4->joint = newRigidJoint("Joint_4", r6_3, theta->data[3][0], theta_dot->data[3][0], theta_ddot->data[3][0], lims, 0, Body_3->rigid, Body_4->rigid);
    union object_u *Joint_5= malloc(sizeof(union object_u));
    Joint_5->joint = newRigidJoint("Joint_5", r6_2, theta->data[4][0], theta_dot->data[4][0], theta_ddot->data[4][0], lims, 0, Body_4->rigid, Body_5->rigid);

    double zero[2] = {0,0};
    union object_u *joint_EE = malloc(sizeof(union object_u));
    joint_EE->joint =  newRigidJoint("joint_EE", zeros(6,1), 0, 0, 0, zero, 0, NULL, EE->rigid);
    Robot *newRobot = malloc(sizeof(Robot));
    newRobot->name = "RigidRandy";

    //{base, Joint_1, Body_1, Jxoint_2, Body_2, Joint_3, Body_3, Joint_4, Body_4, Joint_5, Body_5, joint_EE, EE};
    Object *robotList = malloc(sizeof(Object) * 13);
    robotList[0].object = base;
    robotList[1].object = Joint_1;

    robotList[2].object = Body_1;
    robotList[3].object = Joint_2;
    robotList[4].object = Body_2;
    robotList[5].object = Joint_3;
    robotList[6].object = Body_3;
    robotList[7].object = Joint_4;
    robotList[8].object = Body_4;
    robotList[9].object = Joint_5;
    robotList[10].object = Body_5;
    robotList[11].object = joint_EE;
    robotList[12].object = EE;


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
int main(void){
    matrix *theta = zeros(5,1);
    matrix  *theta_dot = zeros(5,1);
    matrix *theta_ddot = zeros(5,1);
    Robot *rigidRandy = defRigidKin(theta, theta_dot, theta_ddot);


    double dt = 0.025;
    matrix *t1 = matrix_new(1,100);

    for(int i=0; i<100; i++){
        t1->data[0][i] = t1->data[0][i-1] + dt;
    }
    printf("t1\n");
    printMatrix(t1);
    printf("t1 end\n");


    matrix *shape = zeros(5,1);
    shape->data[0][0] = .4;
    shape->data[1][0] = -0.5*0.4;
    shape->data[2][0] = 0.5*0.4;
    shape->data[3][0] = -0.7*0.4;
    shape->data[4][0] = 0.1*-0.4;

    matrix *Theta_DDot = zeros(5,100);
    matrix *math = zeros(1,100);
    for(int i = 0; i < 100; i++){
        math->data[0][i] = (double)sin(M_PI/(double)(dt*100) * t1->data[0][i]);
    }
    Theta_DDot->data[0] = matrix_scalar_mul(math, shape->data[0][0])->data[0];//todo get rid of 'magic' 100
    Theta_DDot->data[1] = matrix_scalar_mul(math, shape->data[1][0])->data[0];
    Theta_DDot->data[2] = matrix_scalar_mul(math, shape->data[2][0])->data[0];
    Theta_DDot->data[3] = matrix_scalar_mul(math, shape->data[3][0])->data[0];
    Theta_DDot->data[4] = matrix_scalar_mul(math, shape->data[4][0])->data[0];

    printMatrix(Theta_DDot);

    //printMatrix(plotRobotConfig(rigidRandy, theta, 100));

    //printMatrix(Theta_DDot);
    for(int i = 0; i < 100; i++) {
        //printf("Theta\n");
        //printMatrix(theta);
//        printf("\nTheta_DDot\n");
        //printMatrix(Theta_DDot);
        matrixToFile(plotRobotConfig(rigidRandy, theta, 100), "RigidRandyPlot.csv");
        printf("Theta\n");
        printMatrix(theta);
        theta = matrix_add(matrix_scalar_mul(getSection(theta_dot, 0, 4, 0, 0), dt), theta);
        theta_dot = matrix_add(matrix_scalar_mul(getSection(Theta_DDot, 0, 4, i, i), dt), theta_dot);

    }
        return 0;
}