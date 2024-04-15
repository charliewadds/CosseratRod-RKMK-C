#include <stdio.h>
#include <assert.h>
#include "RobotLib.h"
#include "Matrices.h"
#include "LieGroup.h"


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
    matrix F_0 = *zeros(6,1);
    F_0.data[2][0] = 1;
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
    Body_2->object->flex = newFlexBody("Body_2", Mf,  K,C, zeros(6,1), N, L_0);

    Object *Body_3 =  malloc(sizeof(union object_u));
    Body_3->type = 0;
    Body_3->object = malloc(sizeof(union object_u));
    Body_3->object->rigid = newRigidBody("Body_3", M,  linkTwist, linkCoM);

    Object *Body_4 =  malloc(sizeof(union object_u));
    Body_4->type = 1;
    Body_4->object = malloc(sizeof(union object_u));
    Body_4->object->flex = newFlexBody("Body_4",Mf,  K,C, zeros(6,1), N, L_0);

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



    Body_2->object->flex->eta_prev = zeros(6,Body_2->object->flex->N);
    Body_2->object->flex->eta_pprev = zeros(6,Body_2->object->flex->N);

    Body_4->object->flex->eta_prev = zeros(6,Body_4->object->flex->N);
    Body_4->object->flex->eta_pprev = zeros(6,Body_4->object->flex->N);
    //Body_2->object->flex->f_prev



    Body_2->object->flex->f_prev->data[2] = ones(1,6)->data[0];
    Body_2->object->flex->f_pprev->data[2] = ones(1,6)->data[0];

    Body_4->object->flex->f_prev->data[2] = ones(1,6)->data[0];
    Body_4->object->flex->f_pprev->data[2] = ones(1,6)->data[0];




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

int main() {
    matrix *theta_ddot = zeros(5,1);
    theta_ddot->data[0][0] = 0.0126;
    theta_ddot->data[1][0] = -0.0063;
    theta_ddot->data[2][0] = 0.0063;
    theta_ddot->data[3][0] = -0.0088;
    theta_ddot->data[4][0] = -0.0013;

    matrix *init_theta = zeros(6,1);
    init_theta->data[2][0] = 1;
    Robot *robot = defPaperSample_2(zeros(5,1), zeros(5,1), zeros(5,1));

    matrix *out = find_roots(init_theta, robot, zeros(5,1), zeros(5,1), theta_ddot, zeros(6,1), 60, -80, 20);
    //matrix *out2 = find_roots_PSO(init_theta, robot, zeros(5,1), zeros(5,1), theta_ddot, zeros(6,1), 60, -80, 20);
    printMatrix(out);
    return 0;
}