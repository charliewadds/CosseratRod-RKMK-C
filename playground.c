#include <stdio.h>
#include <assert.h>
#include "RobotLib.h"
#include "Matrices.h"
#include "LieGroup.h"
for (int i = 1; i < numBody + 2; i++) {
rigidJoint *joint = robot->objects[2 * (i - 1) + 1]->object->joint;
Object *body = robot->objects[2 * i ];
assert(robot->objects[2 * (i - 1) + 1]->type == 2);
assert(robot->objects[2 * i - 2]->type == 1 || robot->objects[2 * i - 2]->type == 0);

CoM2CoM = getCoM2CoM(joint, CoM2CoM);
//        printf("CoM2CoM\n");
//        printMatrix(CoM2CoM);
//        printf("g_ref[i - 1]");
//        printMatrix(g_ref[i - 1]);
kin = actuateRigidJoint(g_ref[i - 1], CoM2CoM, joint,
                        getSection(eta, 0, 5, i - 1, i - 1), getSection(d_eta, 0, 5, i - 1, i - 1));
g_act_wrt_prev[i] = kin->g_act_wrt_prev;
g_ref[i] = kin->g_cur;

setSection(eta, 0, 5, i, i, kin->eta);
setSection(d_eta, 0, 5, i, i, kin->d_eta);

//        printf("gref-1\n");
//        printMatrix(g_ref[i-1]);
//
//        printf("gref\n");
//        printMatrix(g_ref[i]);
F_temp = matMult(matrix_transpose(adj(g_act_wrt_prev[i])), F_temp);

if (body->type == 1) {//flexible body
if (i == BC_Start + 1) {//todo double check this +1
setSection(F, 0, 5, i, i, matMult(body->object->flex->stiff,
matrix_sub(InitGuess, body->object->flex->F_0)));
} else {
setSection(F, 0, 5, i, i, F_temp);
}
F_dist = zeros(6, body->object->flex->N);
//todo F is wrong here because it comes from InitGuess which comes from the solver which does not work
dyn = flex_dyn(g_ref[i], F_dist, getSection(F, 0, 5, i, i), body->object->flex,
               getSection(eta, 0, 5, i, i), c0, c1, c2);

g_ref[i] = dyn->g_end;
F_temp = matMult(body->object->flex->stiff,
                 matrix_sub(getSection(dyn->f, 0, 5, dyn->f->numCols - 1, dyn->f->numCols - 1),
                            body->object->flex->F_0));


setSection(eta, 0, 5, eta->numCols, eta->numCols,
getSection(dyn->eta, 0, 5, dyn->eta->numCols, dyn->eta->numCols));


body->object->flex->f_pprev = body->object->flex->f_prev;
body->object->flex->f_prev = dyn->f;

body->object->flex->eta_pprev = body->object->flex->eta_prev;//double check this
body->object->flex->eta_prev = dyn->eta;


} else if (i > BC_Start) {//rigid bodies



setSection(F, 0, 5, i-1, i-1, F_temp);// [N;Nm] Save Wrench Between i,i-1_th Body @ CoM Expressed in BCF

if (i < numBody + 2) {

//setSection(C, 0,5, i - 1, i - 1, matMult(matrix_transpose(getSection(F, 0,5,i,i)), robot->objects[2*i-2]->object->joint->twistR6));
for (int j = 0; j < C->numRows; j++) {

setSection(C, 0, C->numRows - 1, i - 1, i - 1,
matrix_outerProduct(  matrix_transpose(getSection(F, 0, 5, i, i)),joint->twistR6));

}
}

if (body->type == 1) {//flex
F_temp = matrix_sub(matrix_add(getSection(F, 0, 5, i, i),
                               matMult(matMult(matrix_transpose(adj_R6(getSection(eta, 0, 5, i, i))),
                                               body->object->flex->mass), getSection(eta, 0, 5, i, i))),
                    matMult(body->object->flex->mass, getSection(eta, 0, 5, i, i)));

//                F_temp = matMult(matrix_add(getSection(F, 0, 5, i, i), matrix_transpose(adj_R6(getSection(eta, 0, 5, i, i)))),
//                        matMult(curr_body->object->flex->mass, getSection(eta, 0, 5, i, i)));
} else {
F_temp = matrix_sub(matrix_add(getSection(F, 0, 5, i, i),
                               matMult(matMult(matrix_transpose(adj_R6(getSection(eta, 0, 5, i, i))),
                                               body->object->rigid->mass), getSection(eta, 0, 5, i, i))),
                    matMult(body->object->rigid->mass, getSection(eta, 0, 5, i, i)));


}

}

}

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





    for (int i = 1; i < numBody + 2; i++) {
        rigidJoint *joint = robot->objects[2 * (i - 1) + 1]->object->joint;
        Object *body = robot->objects[2 * i ];
        assert(robot->objects[2 * (i - 1) + 1]->type == 2);
        assert(robot->objects[2 * i - 2]->type == 1 || robot->objects[2 * i - 2]->type == 0);

        CoM2CoM = getCoM2CoM(joint, CoM2CoM);
//        printf("CoM2CoM\n");
//        printMatrix(CoM2CoM);
//        printf("g_ref[i - 1]");
//        printMatrix(g_ref[i - 1]);
        kin = actuateRigidJoint(g_ref[i - 1], CoM2CoM, joint,
                                getSection(eta, 0, 5, i - 1, i - 1), getSection(d_eta, 0, 5, i - 1, i - 1));
        g_act_wrt_prev[i] = kin->g_act_wrt_prev;
        g_ref[i] = kin->g_cur;

        setSection(eta, 0, 5, i, i, kin->eta);
        setSection(d_eta, 0, 5, i, i, kin->d_eta);

//        printf("gref-1\n");
//        printMatrix(g_ref[i-1]);
//
//        printf("gref\n");
//        printMatrix(g_ref[i]);
        F_temp = matMult(matrix_transpose(adj(g_act_wrt_prev[i])), F_temp);

        if (body->type == 1) {//flexible body
            if (i == BC_Start + 1) {//todo double check this +1
                setSection(F, 0, 5, i, i, matMult(body->object->flex->stiff,
                                                  matrix_sub(InitGuess, body->object->flex->F_0)));
            } else {
                setSection(F, 0, 5, i, i, F_temp);
            }
            F_dist = zeros(6, body->object->flex->N);
            //todo F is wrong here because it comes from InitGuess which comes from the solver which does not work
            dyn = flex_dyn(g_ref[i], F_dist, getSection(F, 0, 5, i, i), body->object->flex,
                           getSection(eta, 0, 5, i, i), c0, c1, c2);

            g_ref[i] = dyn->g_end;
            F_temp = matMult(body->object->flex->stiff,
                             matrix_sub(getSection(dyn->f, 0, 5, dyn->f->numCols - 1, dyn->f->numCols - 1),
                                        body->object->flex->F_0));


            setSection(eta, 0, 5, eta->numCols, eta->numCols,
                       getSection(dyn->eta, 0, 5, dyn->eta->numCols, dyn->eta->numCols));


            body->object->flex->f_pprev = body->object->flex->f_prev;
            body->object->flex->f_prev = dyn->f;

            body->object->flex->eta_pprev = body->object->flex->eta_prev;//double check this
            body->object->flex->eta_prev = dyn->eta;


        } else if (i > BC_Start) {//rigid bodies



            setSection(F, 0, 5, i-1, i-1, F_temp);// [N;Nm] Save Wrench Between i,i-1_th Body @ CoM Expressed in BCF

            if (i < numBody + 2) {

                //setSection(C, 0,5, i - 1, i - 1, matMult(matrix_transpose(getSection(F, 0,5,i,i)), robot->objects[2*i-2]->object->joint->twistR6));
                for (int j = 0; j < C->numRows; j++) {

                    setSection(C, 0, C->numRows - 1, i - 1, i - 1,
                               matrix_outerProduct(  matrix_transpose(getSection(F, 0, 5, i, i)),joint->twistR6));

                }
            }

            if (body->type == 1) {//flex
                F_temp = matrix_sub(matrix_add(getSection(F, 0, 5, i, i),
                                               matMult(matMult(matrix_transpose(adj_R6(getSection(eta, 0, 5, i, i))),
                                                               body->object->flex->mass), getSection(eta, 0, 5, i, i))),
                                    matMult(body->object->flex->mass, getSection(eta, 0, 5, i, i)));

//                F_temp = matMult(matrix_add(getSection(F, 0, 5, i, i), matrix_transpose(adj_R6(getSection(eta, 0, 5, i, i)))),
//                        matMult(curr_body->object->flex->mass, getSection(eta, 0, 5, i, i)));
            } else {
                F_temp = matrix_sub(matrix_add(getSection(F, 0, 5, i, i),
                                               matMult(matMult(matrix_transpose(adj_R6(getSection(eta, 0, 5, i, i))),
                                                               body->object->rigid->mass), getSection(eta, 0, 5, i, i))),
                                    matMult(body->object->rigid->mass, getSection(eta, 0, 5, i, i)));


            }

        }

    }



    return 0;
}