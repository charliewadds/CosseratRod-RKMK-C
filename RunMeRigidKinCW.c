//
// Created by Charlie Wadds on 2024-03-17.
//

#include "RobotLib.h"
#include <stdio.h>
#include <math.h>
#include <float.h>


Robot *defRigidKin(double *theta, double theta_dot, double theta_ddot){
    double linkMass = 12.5;
    double linkLen = 0.5;

    matrix *linkTwist = zeros(6,1);
    linkTwist->data[2][0] = 1;
    linkTwist = matrix_scalar_mul(linkTwist, linkLen);

    matrix *linkTwist_SE3 = linkTwist;

    matrix *linkCoM = elemDiv(linkTwist, 2);

    matrix *linkInertia = matrix_scalar_mul(eye(3), 1.0/12.0 * linkMass * pow(linkLen, 2));

    matrix *M = eye(6);
    setSection(M, 0, 2, 0, 2, matrix_scalar_mul(eye(3),linkMass));
    setSection(M, 3, 5, 3, 5, linkInertia);

    matrix *Z = zeros(6,1);

    SE3 *Z_SE3 = new_SE3_zeros();

    //todo find a better way to do this, maybe json or something
    rigidBody *base = (Object *) newRigidBody("base", matrix_scalar_mul(eye(6), DBL_MAX), Z, Z);//todo dbl max to replace inf
    rigidBody *Body_1 = (Object *) newRigidBody("Body_1", M,  linkTwist_SE3, linkCoM);
    rigidBody *Body_2 = (Object *) newRigidBody("Body_2", M,  linkTwist_SE3, linkCoM);
    rigidBody *Body_3 = (Object *) newRigidBody("Body_3", M,  linkTwist_SE3, linkCoM);
    rigidBody *Body_4 = (Object *) newRigidBody("Body_4", M,  linkTwist_SE3, linkCoM);
    rigidBody *Body_5 = (Object *) newRigidBody("Body_5", M,  linkTwist_SE3, linkCoM);
    rigidBody *EE = (Object *) newRigidBody("EE", zeros(6,6),  Z, Z);


    matrix *r6_2 = zeros(6,1);
    r6_2->data[2][0] = 1;

    matrix *r6_3 = zeros(6,1);
    r6_3->data[3][0] = 1;

    matrix *r6_4 = zeros(6,1);
    r6_4->data[4][0] = 1;

    matrix *r6_5 = zeros(6,1);
    r6_5->data[5][0] = 1;

    double *lims = malloc(sizeof(double) * 2);
    lims[0] = -PI;
    lims[1] = PI;


    double pihalf = PI/2;

    rigidJoint *Joint_1 = newRigidJoint("Joint_1", r6_5, theta[0], theta_dot, theta_ddot, lims, 0, base, Body_1);
    rigidJoint *Joint_2 = newRigidJoint("Joint_2", r6_3, theta[1], theta_dot, theta_ddot, lims, pihalf, Body_1, Body_2);
    rigidJoint *Joint_3 = newRigidJoint("Joint_3", r6_5, theta[2], theta_dot, theta_ddot, lims, 0, Body_2, Body_3);
    rigidJoint *Joint_4 = newRigidJoint("Joint_4", r6_3, theta[3], theta_dot, theta_ddot, lims, 0, Body_3, Body_4);
    rigidJoint *Joint_5 = newRigidJoint("Joint_5", r6_2, theta[4], theta_dot, theta_ddot, lims, 0, Body_4, Body_5);

    rigidJoint *joint_EE = (Object *) newRigidJoint("joint_EE", zeros(6,1), 0, 0, 0, zeros(1,2), 0, NULL, EE);
    Robot *newRobot = malloc(sizeof(Robot));
    newRobot->name = "RigidRandy";

    //{base, Joint_1, Body_1, Joint_2, Body_2, Joint_3, Body_3, Joint_4, Body_4, Joint_5, Body_5, joint_EE, EE};
    Object *robotList = malloc(sizeof(Object) * 13);
    robotList[0].rigid = base;
    robotList[1].joint = Joint_1;
    robotList[2].rigid = Body_1;
    robotList[3].joint = Joint_2;
    robotList[4].rigid = Body_2;
    robotList[5].joint = Joint_3;
    robotList[6].rigid = Body_3;
    robotList[7].joint = Joint_4;
    robotList[8].rigid = Body_4;
    robotList[9].joint = Joint_5;
    robotList[10].rigid = Body_5;
    robotList[11].joint = joint_EE;
    robotList[12].rigid = EE;


    newRobot->objects = robotList;

    newRobot->numObjects = 13;


    return newRobot;
}
int main(void){
    double theta[5] = {0,0,0,0,0};
    double theta_dot = 0;
    double theta_ddot = 0;
    Robot *rigidRandy = defRigidKin(theta, theta_dot, theta_ddot);
    printMatrix(plotRobotConfig(rigidRandy, theta, 100));
    return 0;
}