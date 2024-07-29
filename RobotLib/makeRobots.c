//
// Created by Charlie Wadds on 2024-07-09.
//

#include "RobotLib.h"
#include <assert.h>

Robot *defPaperSample_2(matrix *theta, matrix *theta_dot, matrix *theta_ddot){
    assert(theta->numCols == 1);//todo add asserts like this to all functions with matrix args
    double linkMass = 12.5;
    double linkLen = 0.5;

    matrix *temp3x3n1 = matrix_new(3,3);

    matrix *temp6x6n1 = matrix_new(6,6);


    matrix *tempR6n1 = matrix_new(6,1);
    matrix *tempR6n2 = matrix_new(6,1);
    matrix *linkTwist;




    linkTwist = matrix_new(6,1);
    linkTwist->data[(2 * linkTwist->numCols) + 0] = 1;
    matrix_scalar_mul(linkTwist, linkLen, linkTwist);



    matrix *linkCoM = matrix_new(6,1);
    elemDiv(linkTwist, 2, linkCoM);

    matrix *linkInertia = matrix_new(3,3);
    matrix_scalar_mul(eye(temp3x3n1), 1.0/12.0 * linkMass * pow(linkLen, 2), linkInertia);

    matrix *M = matrix_new(6,6);
    eye(M);

    setSection(M, 0, 2, 0, 2, matrix_scalar_mul(eye(temp3x3n1),linkMass, temp3x3n1));
    setSection(M, 3, 5, 3, 5, linkInertia);

    matrix *Z = zeros(6,1);

    //SE3 *Z_SE3 = new_SE3_zeros();

    double L_0 = 0.4;
    matrix *F_0 = zeros(6,1);
    F_0->data[(2 * F_0->numCols) + 0] = 1;

    double rho = 75e1;
    double mu = 1e5;
    double r = 0.1;
    double E = 1e9;
    double G = E/(2*(1+0.3));
    double I = M_PI/4*pow(r,4);
    double A = M_PI*pow(r,2);
    int N = 21;

    //diag(2I,I,I)
    matrix *J = zeros(3,3);
    J->data[(0 * J->numCols) + 0] = 2*I;
    J->data[(1 * J->numCols) + 1] = I;
    J->data[(2 * J->numCols) + 2] = I;

    matrix *Kbt = zeros(3,3);
    Kbt->data[(0 * Kbt->numCols) + 0] = 2*G*I;
    Kbt->data[(1 * Kbt->numCols) + 1] = E*I;
    Kbt->data[(2 * Kbt->numCols) + 2] = E*I;

    matrix *Kse = zeros(3,3);
    Kse->data[(0 * Kse->numCols) + 0] = E*A;
    Kse->data[(1 * Kse->numCols) + 1] = G*A;
    Kse->data[(2 * Kse->numCols) + 2] = G*A;

    matrix *Cse = zeros(3,3);
    Cse->data[(0 * Cse->numCols) + 0] = 3*A;
    Cse->data[(1 * Cse->numCols) + 1] = A;
    Cse->data[(2 * Cse->numCols) + 2] = A;
    matrix_scalar_mul(Cse, mu, Cse);

    matrix *Cbt = zeros(3,3);
    Cbt->data[(0 * Cbt->numCols) + 0] = 2*I;
    Cbt->data[(1 * Cbt->numCols) + 1] = I;
    Cbt->data[(2 * Cbt->numCols) + 2] = I;
    matrix_scalar_mul(Cbt, mu, Cbt);

    matrix *K = zeros(6,6);
    setSection(K, 0, 2, 0, 2, Kse);
    setSection(K, 3, 5, 3, 5, Kbt);

    matrix *C = zeros(6,6);
    setSection(C, 0, 2, 0, 2, Cse);
    setSection(C, 3, 5, 3, 5, Cbt);


    matrix *Mf = zeros(6,6);
    setSection(Mf, 0, 2, 0, 2, matrix_scalar_mul(eye(temp3x3n1),rho*A, temp3x3n1));
    setSection(Mf, 3, 5, 3, 5, matrix_scalar_mul(J,rho, temp3x3n1));




    //todo find a better way to do this, maybe json or something
    Object *base = malloc(sizeof(struct object_s));
    base->type = 0;
    base->object = malloc(sizeof(union object_u));
    base->object->rigid = newRigidBody("base", matrix_scalar_mul(eye(temp6x6n1), INFINITY, temp6x6n1), Z, Z);//todo dbl max to replace inf

    Object *Body_1 =  malloc(sizeof(struct object_s));
    Body_1->type = 0;
    Body_1->object = malloc(sizeof(union object_u));
    Body_1->object->rigid = newRigidBody("Body_1", M,  linkTwist, linkCoM);

    Object *Body_2 =  malloc(sizeof(struct object_s));
    Body_2->type = 1;
    Body_2->object = malloc(sizeof(union object_u));
    Body_2->object->flex = newFlexBody("Body_2", Mf,  K,C, F_0, N, L_0);



    Object *Body_3 =  malloc(sizeof(struct object_s));
    Body_3->type = 0;
    Body_3->object = malloc(sizeof(union object_u));
    Body_3->object->rigid = newRigidBody("Body_3", M,  linkTwist, linkCoM);

    Object *Body_4 =  malloc(sizeof(struct object_s));
    Body_4->type = 1;
    Body_4->object = malloc(sizeof(union object_u));
    Body_4->object->flex = newFlexBody("Body_4",Mf,  K,C, F_0, N, L_0);
    Body_4->object->flex->F_0->data[(2 * Body_4->object->flex->F_0->numCols) + 0] = 1;

    Object *Body_5 =  malloc(sizeof(struct object_s));
    Body_5->type = 0;
    Body_5->object = malloc(sizeof(union object_u));
    Body_5->object->rigid = newRigidBody("Body_5", matrix_scalar_mul(M,0.5, temp6x6n1), matrix_scalar_mul(linkTwist,0.5, tempR6n1), matrix_scalar_mul(linkCoM,0.5, tempR6n2));

    Object *EE =      malloc(sizeof(struct object_s));
    EE->type = 0;
    EE->object = malloc(sizeof(union object_u));
    EE->object->rigid = newRigidBody("EE", zeros(6,6),  Z, Z);


    matrix *r6_2 = zeros(6,1);
    r6_2->data[2] = 1;

    matrix *r6_3 = zeros(6,1);
    r6_3->data[3] = 1;

    matrix *r6_4 = zeros(6,1);
    r6_4->data[4] = 1;

    matrix *r6_5 = zeros(6,1);
    r6_5->data[5] = 1;

    double *lims = malloc(sizeof(double) * 2);
    lims[0] = -M_PI;
    lims[1] = M_PI;



    //Body_2->object->flex->eta_prev = zeros(6,Body_2->object->flex->N);
    //Body_2->object->flex->eta_pprev = zeros(6,Body_2->object->flex->N);

    //Body_4->object->flex->eta_prev = zeros(6,Body_4->object->flex->N);
    //Body_4->object->flex->eta_pprev = zeros(6,Body_4->object->flex->N);
    //Body_2->object->flex->f_prev


    for(int i = 0; i < N; i++){
        Body_2->object->flex->f_prev->data[2 * N + i] = 1;
        Body_2->object->flex->f_pprev->data[2 * N + i] = 1;
        Body_4->object->flex->f_prev->data[2 * N + i] = 1;
        Body_4->object->flex->f_pprev->data[2 * N + i] = 1;
    }

    double pihalf = M_PI/2;

    Object **robotList = calloc(13, sizeof(Object));

    for (int i = 0; i < 13; i++){
        robotList[i] = malloc(sizeof(Object));
    }
    //todo this should be in a function like createObject or something

    robotList[1]->type = 2;
    robotList[1]->object = malloc(sizeof(union object_u));

    robotList[1]->object->joint = newRigidJoint("Joint_1", r6_5, theta->data[(0 * theta->numCols) + 0], theta_dot->data[(0 * theta_dot->numCols) + 0], theta_ddot->data[(0 * theta_ddot->numCols) + 0], lims, 0, base, Body_1);

    //Object *Joint_2 = malloc(sizeof(Object));
    robotList[3]->type = 2;
    robotList[3]->object = malloc(sizeof(union object_u));
    robotList[3]->object->joint = newRigidJoint("Joint_2", r6_3, theta->data[(1 * theta->numCols) + 0], theta_dot->data[(1 * theta_dot->numCols) + 0], theta_ddot->data[(1 * theta_ddot->numCols) + 0], lims, pihalf, Body_1, Body_2);

    //Object *Joint_3= malloc(sizeof(Object));
    robotList[5]->type = 2;
    robotList[5]->object = malloc(sizeof(union object_u));
    robotList[5]->object->joint = newRigidJoint("Joint_3", r6_4, theta->data[(2 * theta->numCols) + 0], theta_dot->data[(2 * theta_dot->numCols) + 0], theta_ddot->data[(2 * theta_ddot->numCols) + 0], lims, 0, Body_2, Body_3);

    //Object *Joint_4= malloc(sizeof(Object));
    robotList[7]->type = 2;
    robotList[7]->object = malloc(sizeof(union object_u));
    robotList[7]->object->joint = newRigidJoint("Joint_4", r6_3, theta->data[(3 * theta->numCols) + 0], theta_dot->data[(3 * theta_dot->numCols) + 0], theta_ddot->data[(3 * theta_ddot->numCols) + 0], lims, 0, Body_3, Body_4);

    //Object *Joint_5 = malloc(sizeof(Object));
    robotList[9]->type = 2;
    robotList[9]->object = malloc(sizeof(union object_u));
    robotList[9]->object->joint = newRigidJoint("Joint_5", r6_2, theta->data[(4 * theta->numCols) + 0], theta_dot->data[(4 * theta_dot->numCols) + 0], theta_ddot->data[(4 * theta_ddot->numCols) + 0], lims, 0, Body_4, Body_5);

    double zero[2] = {0,0};
    zeroMatrix(r6_2);//this is just so I dont need to create another matrix
    //Object *joint_EE = malloc(sizeof(struct object_s));
    robotList[11]->type = 2;
    robotList[11]->object = malloc(sizeof(union object_u));
    robotList[11]->object->joint =  newRigidJoint("joint_EE", zeros(6,1), 0, 0, 0, zero, 0, Body_5, EE);

    Robot *newRobot = malloc(sizeof(Robot));
    newRobot->name = "RigidRandy";

    //{base, Joint_1, Body_1, Jxoint_2, Body_2, Joint_3, Body_3, Joint_4, Body_4, Joint_5, Body_5, joint_EE, EE};

    robotList[0] = base;
    //robotList[1] = Joint_1;
    robotList[2] = Body_1;
    //robotList[3] = Joint_2;
    robotList[4] = Body_2;
    //robotList[5] = Joint_3;
    robotList[6] = Body_3;
    //robotList[7] = Joint_4;
    robotList[8] = Body_4;
    //robotList[9] = Joint_5;
    robotList[10] = Body_5;
    //robotList[11] = joint_EE;
    robotList[12] = EE;



    newRobot->objects = robotList;
    newRobot->numBody = 5;
    newRobot->BC_Start = 2;
    newRobot->BC_End = 4;
    newRobot->numObjects = 13;


    matrix_free(linkTwist);
    matrix_free(linkCoM);
    matrix_free(linkInertia);
    matrix_free(Z);
    matrix_free(J);
    matrix_free(Kbt);
    matrix_free(Kse);
    matrix_free(Cse);
    matrix_free(Cbt);
    matrix_free(F_0);
    matrix_free(r6_2);
    matrix_free(r6_3);
    matrix_free(r6_4);
    matrix_free(r6_5);
    matrix_free(M);
    matrix_free(K);
    matrix_free(C);
    matrix_free(Mf);

    matrix_free(temp3x3n1);
    matrix_free(temp6x6n1);
    matrix_free(tempR6n1);
    matrix_free(tempR6n2);

    return newRobot;
}


Robot *defPaperSample_1(matrix *theta, matrix *theta_dot, matrix *theta_ddot) {
    double linkMass = 0.01;
    double linkLen = 0.2;

    matrix *temp3x3n1 = matrix_new(3,3);

    matrix *temp6x6n1 = matrix_new(6,6);


    matrix *tempR6n1 = matrix_new(6,1);
    matrix *tempR6n2 = matrix_new(6,1);
    matrix *linkTwist;




    linkTwist = matrix_new(6,1);
    linkTwist->data[(2 * linkTwist->numCols) + 0] = 1;
    matrix_scalar_mul(linkTwist, linkLen, linkTwist);



    matrix *linkCoM = matrix_new(6,1);
    elemDiv(linkTwist, 2, linkCoM);

    matrix *linkInertia = matrix_new(3,3);
    matrix_scalar_mul(eye(temp3x3n1), 1.0/12.0 * linkMass * pow(linkLen, 2), linkInertia);

    matrix *M = matrix_new(6,6);
    eye(M);

    setSection(M, 0, 2, 0, 2, matrix_scalar_mul(eye(temp3x3n1),linkMass, temp3x3n1));
    setSection(M, 3, 5, 3, 5, linkInertia);

    matrix *Z = zeros(6,1);

    //SE3 *Z_SE3 = new_SE3_zeros();

    double L_0 = 0.4;
    matrix *F_0 = zeros(6,1);
    F_0->data[(2 * F_0->numCols) + 0] = 1;

    double rho = 75e1;
    double mu = 5e5;
    double r = 0.01;
    double E = 1e8;
    double G = E/(2*(1+0.3));
    double I = M_PI/4*pow(r,4);
    double A = M_PI*pow(r,2);
    int N = 21;


    //diag(2I,I,I)
    matrix *J = zeros(3,3);
    J->data[(0 * J->numCols) + 0] = 2*I;
    J->data[(1 * J->numCols) + 1] = I;
    J->data[(2 * J->numCols) + 2] = I;

    matrix *Kbt = zeros(3,3);
    Kbt->data[(0 * Kbt->numCols) + 0] = 2*G*I;
    Kbt->data[(1 * Kbt->numCols) + 1] = E*I;
    Kbt->data[(2 * Kbt->numCols) + 2] = E*I;

    matrix *Kse = zeros(3,3);
    Kse->data[(0 * Kse->numCols) + 0] = E*A;
    Kse->data[(1 * Kse->numCols) + 1] = G*A;
    Kse->data[(2 * Kse->numCols) + 2] = G*A;

    matrix *Cse = zeros(3,3);
    Cse->data[(0 * Cse->numCols) + 0] = 3*A;
    Cse->data[(1 * Cse->numCols) + 1] = A;
    Cse->data[(2 * Cse->numCols) + 2] = A;
    matrix_scalar_mul(Cse, mu, Cse);

    matrix *Cbt = zeros(3,3);
    Cbt->data[(0 * Cbt->numCols) + 0] = 2*I;
    Cbt->data[(1 * Cbt->numCols) + 1] = I;
    Cbt->data[(2 * Cbt->numCols) + 2] = I;
    matrix_scalar_mul(Cbt, mu, Cbt);

    matrix *K = zeros(6,6);
    setSection(K, 0, 2, 0, 2, Kse);
    setSection(K, 3, 5, 3, 5, Kbt);

    matrix *C = zeros(6,6);
    setSection(C, 0, 2, 0, 2, Cse);
    setSection(C, 3, 5, 3, 5, Cbt);


    matrix *Mf = zeros(6,6);
    setSection(Mf, 0, 2, 0, 2, matrix_scalar_mul(eye(temp3x3n1),rho*A, temp3x3n1));
    setSection(Mf, 3, 5, 3, 5, matrix_scalar_mul(J,rho, temp3x3n1));


    Object *base = malloc(sizeof(struct object_s));
    base->type = 0;
    base->object = malloc(sizeof(union object_u));
    base->object->rigid = newRigidBody("base", matrix_scalar_mul(eye(temp6x6n1), INFINITY, temp6x6n1), Z, Z);

    Object *Body_1 =  malloc(sizeof(struct object_s));
    Body_1->type = 1;
    Body_1->object = malloc(sizeof(union object_u));
    Body_1->object->flex = newFlexBody("Body_1", Mf,  K,C, F_0, N, L_0);


    Object *Body_2 =  malloc(sizeof(struct object_s));
    Body_2->type = 0;
    Body_2->object = malloc(sizeof(union object_u));
    Body_2->object->rigid = newRigidBody("Body_2", M,  linkTwist, linkCoM);




    Object *Body_3 =  malloc(sizeof(struct object_s));
    Body_3->type = 1;
    Body_3->object = malloc(sizeof(union object_u));
    Body_3->object->flex = newFlexBody("Body_3", Mf,  K,C, F_0, N, L_0);

    Object *EE =      malloc(sizeof(struct object_s));
    EE->type = 0;
    EE->object = malloc(sizeof(union object_u));
    EE->object->rigid = newRigidBody("EE", zeros(6,6),  Z, Z);

    matrix *r6_2 = zeros(6,1);
    r6_2->data[(2 * r6_2->numCols) + 0] = 1;

    matrix *r6_3 = zeros(6,1);
    r6_3->data[(3 * r6_3->numCols) + 0] = 1;

    matrix *r6_4 = zeros(6,1);
    r6_4->data[(4 * r6_4->numCols) + 0] = 1;

    matrix *r6_5 = zeros(6,1);
    r6_5->data[(5 * r6_5->numCols) + 0] = 1;

    double *lims = malloc(sizeof(double) * 2);
    lims[0] = -M_PI;
    lims[1] = M_PI;



    //Body_2->object->flex->eta_prev = zeros(6,Body_2->object->flex->N);
    //Body_2->object->flex->eta_pprev = zeros(6,Body_2->object->flex->N);

    //Body_4->object->flex->eta_prev = zeros(6,Body_4->object->flex->N);
    //Body_4->object->flex->eta_pprev = zeros(6,Body_4->object->flex->N);
    //Body_2->object->flex->f_prev



    for(int i = 0; i < N; i++){
        Body_1->object->flex->f_prev->data[2 * N + i] = 1;
        Body_1->object->flex->f_pprev->data[2 * N + i] = 1;
        Body_3->object->flex->f_prev->data[2 * N + i] = 1;
        Body_3->object->flex->f_pprev->data[2 * N + i] = 1;
    }

    double pihalf = M_PI/2;

    Object **robotList = calloc(13, sizeof(Object));

    for (int i = 0; i < 13; i++){
        robotList[i] = malloc(sizeof(Object));
    }
    //todo this should be in a function like createObject or something

    robotList[1]->type = 2;
    robotList[1]->object = malloc(sizeof(union object_u));

    robotList[1]->object->joint = newRigidJoint("Joint_1", r6_4, theta->data[(0 * theta->numCols) + 0], theta_dot->data[(0 * theta_dot->numCols) + 0], theta_ddot->data[(0 * theta_ddot->numCols) + 0], lims, 0, base, Body_1);

    //Object *Joint_2 = malloc(sizeof(Object));
    robotList[3]->type = 2;
    robotList[3]->object = malloc(sizeof(union object_u));
    robotList[3]->object->joint = newRigidJoint("Joint_2", r6_4, theta->data[(1 * theta->numCols) + 0], theta_dot->data[(1 * theta_dot->numCols) + 0], theta_ddot->data[(1 * theta_ddot->numCols) + 0], lims, 0, Body_1, Body_2);

    //Object *Joint_3= malloc(sizeof(Object));
    robotList[5]->type = 2;
    robotList[5]->object = malloc(sizeof(union object_u));
    robotList[5]->object->joint = newRigidJoint("Joint_3", r6_4, theta->data[(2 * theta->numCols) + 0], theta_dot->data[(2 * theta_dot->numCols) + 0], theta_ddot->data[(2 * theta_ddot->numCols) + 0], lims, 0, Body_2, Body_3);


    double zero[2] = {0,0};
    zeroMatrix(r6_2);//this is just so I dont need to create another matrix
    //Object *joint_EE = malloc(sizeof(struct object_s));
    robotList[7]->type = 2;
    robotList[7]->object = malloc(sizeof(union object_u));
    robotList[7]->object->joint =  newRigidJoint("joint_EE", zeros(6,1), 0, 0, 0, zero, 0, Body_3, EE);

    Robot *newRobot = malloc(sizeof(Robot));
    newRobot->name = "RigidRandy_1";

    robotList[0] = base;
    //robotList[1] = Joint_1;
    robotList[2] = Body_1;
    //robotList[3] = Joint_2;
    robotList[4] = Body_2;
    //robotList[5] = Joint_3;
    robotList[6] = Body_3;
    //robotList[7] = joint_EE
    robotList[8] = EE;
    newRobot->numObjects = 9;
    newRobot->numBody = 3;
    newRobot->BC_Start = 1;
    newRobot->BC_End = 3;
    newRobot->objects = robotList;
    return newRobot;

}

Robot *defIcraRobot(matrix *theta, matrix *theta_dot, matrix *theta_ddot) {
    double linkMass = 0.01;
    double linkLen = 0.2;

    matrix *temp3x3n1 = matrix_new(3,3);

    matrix *temp6x6n1 = matrix_new(6,6);


    matrix *tempR6n1 = matrix_new(6,1);
    matrix *tempR6n2 = matrix_new(6,1);
    matrix *linkTwist;




    linkTwist = matrix_new(6,1);
    linkTwist->data[(2 * linkTwist->numCols) + 0] = 1;
    matrix_scalar_mul(linkTwist, linkLen, linkTwist);



    matrix *linkCoM = matrix_new(6,1);
    elemDiv(linkTwist, 2, linkCoM);

    matrix *linkInertia = matrix_new(3,3);
    matrix_scalar_mul(eye(temp3x3n1), 1.0/12.0 * linkMass * pow(linkLen, 2), linkInertia);

    matrix *M = matrix_new(6,6);
    eye(M);

    setSection(M, 0, 2, 0, 2, matrix_scalar_mul(eye(temp3x3n1),linkMass, temp3x3n1));
    setSection(M, 3, 5, 3, 5, linkInertia);

    matrix *Z = zeros(6,1);

    //SE3 *Z_SE3 = new_SE3_zeros();

    double L_0 = 0.15;
    matrix *F_0 = zeros(6,1);
    F_0->data[(2 * F_0->numCols) + 0] = 1;

    double rho = 1080;
    double mu = 11;
    double r = 0.015;
    double E = 1e7;
    double G = E/(2*(1+0.3));
    double I = M_PI/4*pow(r,4);
    double A = M_PI*pow(r,2);
    int N = 21;


    //diag(2I,I,I)
    matrix *J = zeros(3,3);
    J->data[(0 * J->numCols) + 0] = 2*I;
    J->data[(1 * J->numCols) + 1] = I;
    J->data[(2 * J->numCols) + 2] = I;

    matrix *Kbt = zeros(3,3);
    Kbt->data[(0 * Kbt->numCols) + 0] = 2*G*I;
    Kbt->data[(1 * Kbt->numCols) + 1] = E*I;
    Kbt->data[(2 * Kbt->numCols) + 2] = E*I;

    matrix *Kse = zeros(3,3);
    Kse->data[(0 * Kse->numCols) + 0] = E*A;
    Kse->data[(1 * Kse->numCols) + 1] = G*A;
    Kse->data[(2 * Kse->numCols) + 2] = G*A;

    matrix *Cse = zeros(3,3);
    Cse->data[(0 * Cse->numCols) + 0] = 3*A;
    Cse->data[(1 * Cse->numCols) + 1] = A;
    Cse->data[(2 * Cse->numCols) + 2] = A;
    matrix_scalar_mul(Cse, mu, Cse);

    matrix *Cbt = zeros(3,3);
    Cbt->data[(0 * Cbt->numCols) + 0] = 2*I;
    Cbt->data[(1 * Cbt->numCols) + 1] = I;
    Cbt->data[(2 * Cbt->numCols) + 2] = I;
    matrix_scalar_mul(Cbt, mu, Cbt);

    matrix *K = zeros(6,6);
    setSection(K, 0, 2, 0, 2, Kse);
    setSection(K, 3, 5, 3, 5, Kbt);

    matrix *C = zeros(6,6);
    setSection(C, 0, 2, 0, 2, Cse);
    setSection(C, 3, 5, 3, 5, Cbt);


    matrix *Mf = zeros(6,6);
    setSection(Mf, 0, 2, 0, 2, matrix_scalar_mul(eye(temp3x3n1),rho*A, temp3x3n1));
    setSection(Mf, 3, 5, 3, 5, matrix_scalar_mul(J,rho, temp3x3n1));


    Object *base = malloc(sizeof(struct object_s));
    base->type = 0;
    base->object = malloc(sizeof(union object_u));
    base->object->rigid = newRigidBody("base", matrix_scalar_mul(eye(temp6x6n1), INFINITY, temp6x6n1), Z, Z);

    Object *Body_1 =  malloc(sizeof(struct object_s));
    Body_1->type = 1;
    Body_1->object = malloc(sizeof(union object_u));
    Body_1->object->flex = newFlexBody("Body_1", Mf,  K,C, F_0, N, L_0);


    Object *Body_3 =  malloc(sizeof(struct object_s));
    Body_3->type = 1;
    Body_3->object = malloc(sizeof(union object_u));
    Body_3->object->flex = newFlexBody("Body_3", Mf,  K,C, F_0, N, L_0);

    Object *EE =      malloc(sizeof(struct object_s));
    EE->type = 0;
    EE->object = malloc(sizeof(union object_u));
    EE->object->rigid = newRigidBody("EE", zeros(6,6),  Z, Z);

    matrix *r6_2 = zeros(6,1);
    r6_2->data[(2 * r6_2->numCols) + 0] = 1;

    matrix *r6_3 = zeros(6,1);
    r6_3->data[(3 * r6_3->numCols) + 0] = 1;

    matrix *r6_4 = zeros(6,1);
    r6_4->data[(4 * r6_4->numCols) + 0] = 1;

    matrix *r6_5 = zeros(6,1);
    r6_5->data[(5 * r6_5->numCols) + 0] = 1;

    double *lims = malloc(sizeof(double) * 2);
    lims[0] = -M_PI;
    lims[1] = M_PI;



    //Body_2->object->flex->eta_prev = zeros(6,Body_2->object->flex->N);
    //Body_2->object->flex->eta_pprev = zeros(6,Body_2->object->flex->N);

    //Body_4->object->flex->eta_prev = zeros(6,Body_4->object->flex->N);
    //Body_4->object->flex->eta_pprev = zeros(6,Body_4->object->flex->N);
    //Body_2->object->flex->f_prev



    for(int i = 0; i < N; i++){
        Body_1->object->flex->f_prev->data[2 * N + i] = 1;
        Body_1->object->flex->f_pprev->data[2 * N + i] = 1;
        Body_3->object->flex->f_prev->data[2 * N + i] = 1;
        Body_3->object->flex->f_pprev->data[2 * N + i] = 1;
    }

    double pihalf = M_PI/2;

    Object **robotList = calloc(13, sizeof(Object));

    for (int i = 0; i < 13; i++){
        robotList[i] = malloc(sizeof(Object));
    }
    //todo this should be in a function like createObject or something

    robotList[1]->type = 2;
    robotList[1]->object = malloc(sizeof(union object_u));

    robotList[1]->object->joint = newRigidJoint("Joint_1", r6_4, theta->data[(0 * theta->numCols) + 0], theta_dot->data[(0 * theta_dot->numCols) + 0], theta_ddot->data[(0 * theta_ddot->numCols) + 0], lims, 0, base, Body_1);

    //Object *Joint_2 = malloc(sizeof(Object));
    robotList[3]->type = 2;
    robotList[3]->object = malloc(sizeof(union object_u));
    robotList[3]->object->joint = newRigidJoint("Joint_2", r6_4, theta->data[(1 * theta->numCols) + 0], theta_dot->data[(1 * theta_dot->numCols) + 0], theta_ddot->data[(1 * theta_ddot->numCols) + 0], lims, 0, Body_1, Body_3);


    double zero[2] = {0,0};
    zeroMatrix(r6_2);//this is just so I dont need to create another matrix
    //Object *joint_EE = malloc(sizeof(struct object_s));
    robotList[5]->type = 2;
    robotList[5]->object = malloc(sizeof(union object_u));
    robotList[5]->object->joint =  newRigidJoint("joint_EE", zeros(6,1), 0, 0, 0, zero, 0, Body_3, EE);

    Robot *newRobot = malloc(sizeof(Robot));
    newRobot->name = "IcraRobot";

    robotList[0] = base;
    //robotList[1] = Joint_1;
    robotList[2] = Body_1;
    //robotList[3] = Joint_2;
    //robotList[5] = Joint_3;
    robotList[4] = Body_3;
    //robotList[7] = joint_EE
    robotList[6] = EE;
    newRobot->numObjects = 7;
    newRobot->numBody = 2;
    newRobot->BC_Start = 1;
    newRobot->BC_End = 2;
    newRobot->objects = robotList;
    return newRobot;

}
