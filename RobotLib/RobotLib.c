//
// Created by Charlie Wadds on 2024-03-01.
//

#include "RobotLib.h"
#include "../levmar-2.6/levmar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_errno.h>

matrix* getCoM2CoM(rigidJoint *joint, matrix *CoM2CoM){
    matrix *parentCoM;
    matrix *childCoM;





    if(joint->parent->type == 0){
        parentCoM = joint->parent->body->rigid->CoM;
    }else if(joint->parent->type == 1){
        parentCoM = joint->parent->body->flex->CoM;
    }else{
        printf("Invalid Body Type (Flex_MB_BCS)\n");
        assert(0);
        ///return zeros(6,1);
    }


    if(joint->child->type == 0){
        childCoM = joint->child->body->rigid->CoM;
    }else if(joint->child->type == 1){
        childCoM = joint->child->body->flex->CoM;
    }else{
        printf("Invalid Body Type (Flex_MB_BCS)\n");
        assert(0);
        //return zeros(6,1);
    }
    assert(parentCoM->numCols == 1);
    assert(parentCoM->numRows == 6);

    assert(childCoM->numCols == 1);
    assert(childCoM->numRows == 6);

    //todo do I really need this many??
    matrix *temp4x4n1 = matrix_new(4,4);
    matrix *temp4x4n2 = matrix_new(4,4);
    matrix *temp4x4n3 = matrix_new(4,4);
    matrix *temp4x4n4 = matrix_new(4,4);

    matrix *tempR6n1 = matrix_new(6,1);
    matrix *tempR6n2 = matrix_new(6,1);
    matrix *tempR6n3 = matrix_new(6,1);
    matrix *tempR6n4 = matrix_new(6,1);

    expm_SE3(hat_R6(matrix_sub(joint->parent->body->rigid->Transform, parentCoM, tempR6n1), temp4x4n1), temp4x4n1);

    matrix_scalar_mul(joint->twistR6, joint->homepos, tempR6n2);
    hat_R6(tempR6n2, temp4x4n2);
    expm_SE3(temp4x4n2, temp4x4n3);

    matMult(
            temp4x4n1,
            temp4x4n3,
            temp4x4n3);

    zeroMatrix(CoM2CoM);
    matMult(temp4x4n3,expm_SE3(hat_R6(childCoM, temp4x4n4), temp4x4n4 ), CoM2CoM);
    //free(parentCoM);
//    matMult(matMult(
//                              //curr_obj
//            expm_SE3(hat_R6(matrix_sub(joint->parent->body->rigid->Transform, parentCoM, tempR6n1), temp4x4n1), temp4x4n1),
//            expm_SE3(hat_R6(matrix_scalar_mul(joint->twistR6, joint->homepos, tempR6n2), temp4x4n2), temp4x4n2), temp4x4n1),
//                      expm_SE3(hat_R6(childCoM, temp4x4n3), temp4x4n3 ), CoM2CoM);
//    //free(parentCoM);
    //free(childCoM);

    matrix_free(temp4x4n1);
    matrix_free(temp4x4n2);
    matrix_free(temp4x4n3);
    matrix_free(temp4x4n4);
    matrix_free(tempR6n1);
    matrix_free(tempR6n2);
    matrix_free(tempR6n3);
    matrix_free(tempR6n4);

    return CoM2CoM;

}


rigidKin *actuateRigidJoint(matrix *g_old, matrix *g_oldToCur, rigidJoint *joint, matrix *eta_old, matrix *d_eta_old, rigidKin *result) {
    /*
     - g_cur:          Transformation from Base Frame to i_th Body CoM BCF in RRC (SE3)
     - g_act_wrt_prev: Transformation from i_th Body CoM BCF to i-1_th Body CoM BCF in RAC (SE3
     - eta:            Twist for Body velocity of i_th CoM expressed in i_th CoM BCF (column vector in R6)
     - d_eta:          Time-Rate of Change of Body Velocity twists of i_th CoM expressed in i_th CoM BCF (column vector in R6)

     - g_old:          Transformation from Base Frame to i-1_th Body CoM BCF in RRC
     - g_old2cur:      Transformation from Old Body CoM to Current Body CoM
     - Joint:          Joint Definition including Twist, Position, Velocity, Acceleration
     - eta_old:        Twist for Body velocity of i-1_th CoM expressed in i-1_th CoM BCF
     - d_eta_old:      Time-Rate of Change of Body Velocity twists of i-1_th CoM expressed in i-1_th CoM BCF
     - RRC:            Robot Reference Configuration
     - RAC:            Robot Actuated Configuration
     - BCF:            Body Coordinate Frame
     */


    assert(d_eta_old->numCols == 1);
    assert(d_eta_old->numRows == 6);

    assert(eta_old->numCols == 1);
    assert(eta_old->numRows == 6);

    assert(g_old->numCols == 4);
    assert(g_old->numRows == 4);

    assert(g_oldToCur->numCols == 4);
    assert(g_oldToCur->numRows == 4);



    //matrix *parentCoM = malloc(sizeof(matrix));
    matrix *childCoM = matrix_new(6,1);

//    if(joint->parent->type == 0){
//
//        parentCoM = joint->parent->body->rigid->CoM;
//    }else if(joint->parent->type == 1){
//
//        parentCoM = joint->parent->body->flex->CoM;
//
//    }

    if(joint->child->type == 0){

        //childCoM = joint->child->body->rigid->CoM;
        copyMatrix(joint->child->body->rigid->CoM, childCoM);
    }else if(joint->child->type == 1){

        //childCoM = joint->child->body->flex->CoM;
        copyMatrix(joint->child->body->flex->CoM, childCoM);

    }

    matrix *temp4x4n1 = matrix_new(4,4);
    //matrix *temp4x4n2 = matrix_new(4,4);

    matrix *temp6x6n1 = matrix_new(6,6);


    hat_R6(childCoM, temp4x4n1);
    matrix_scalar_mul(temp4x4n1,-1, temp4x4n1);
    expm_SE3(temp4x4n1, temp4x4n1);

    adj(temp4x4n1, temp6x6n1);

    matrix *temp6x6n2 = matrix_new(6,6);
    //matrix *temp6x6n3 = matrix_new(6,6);

    matrix *newTwist = matrix_new(6,1);
    matMult(temp6x6n1, joint->twistR6, newTwist);//redefine joint to now be about child CoM


    //temp4x4 and 6x6 are available


    matrix *g_cur = matrix_new(4,4);
    matMult(g_old, g_oldToCur, g_cur);

    matrix *g_cur_wrt_prev = matrix_new(4,4);
            matrix_inverse(g_oldToCur, g_cur_wrt_prev);


    matrix *g_act_wrt_prev = matMult(expm_SE3(matrix_scalar_mul(matrix_scalar_mul(hat_R6(newTwist, temp4x4n1), -1, temp4x4n1), joint->position, temp4x4n1), temp4x4n1), g_cur_wrt_prev, temp4x4n1);


    matrix *temp6x1n1 = matrix_new(6,1);
    matrix *temp6x1n2 = matrix_new(6,1);
    matrix *temp6x1n3 = matrix_new(6,1);

    matrix *eta = matrix_new(6,1);

    matMult( adj(g_act_wrt_prev, temp6x6n1), eta_old, temp6x1n1);


    matrix_scalar_mul(newTwist, joint->velocity, temp6x1n2);
    matrix_add(temp6x1n1, temp6x1n2, eta);

    //assert(hasNan(eta) == 0);
    matrix *d_eta = matrix_new(6,1);
    matrix_add(
            matrix_add(matMult(adj(g_act_wrt_prev, temp6x6n1), d_eta_old, temp6x1n1), matMult(adj_R6(eta, temp6x6n2), matrix_scalar_mul(newTwist, joint->velocity, temp6x1n2), temp6x1n2), temp6x1n2),
                               matrix_scalar_mul(newTwist, joint->acceleration, temp6x1n3), d_eta);

    //rigidKin *kin =  malloc(sizeof(rigidKin));
    //memcpy(result->g_cur, g_cur, sizeof(matrix));
    copyMatrix(g_cur, result->g_cur);
    //result->g_cur = g_cur;
    //memcpy(result->g_act_wrt_prev, g_cur_wrt_prev, sizeof(matrix));
    copyMatrix(g_act_wrt_prev, result->g_act_wrt_prev);
    //result->g_act_wrt_prev = g_act_wrt_prev;
    //memcpy(result->eta, eta, sizeof(matrix));
    copyMatrix(eta, result->eta);
    //result->eta = eta;
    //memcpy(result->d_eta, d_eta, sizeof(matrix));
    //result->d_eta = d_eta;
    copyMatrix(d_eta, result->d_eta);

    matrix_free(eta);
    matrix_free(childCoM);

    matrix_free(newTwist);
    matrix_free(g_cur);
    matrix_free(g_cur_wrt_prev);//todo this might already be freed
    matrix_free(temp6x6n1);
    matrix_free(temp6x6n2);
    matrix_free(temp4x4n1);
    matrix_free(d_eta);

    matrix_free(temp6x1n1);
    matrix_free(temp6x1n2);
    matrix_free(temp6x1n3);
    return result;
}

rigidBody *rigidBody_alloc(){
    rigidBody *body = (rigidBody *) malloc(sizeof(rigidBody));
    body->name = malloc(100);
    body->mass = matrix_new(6,6);
    body->Transform = matrix_new(6,1);
    body->CoM = matrix_new(6,1);
    return body;
}

rigidBody *newRigidBody(char *name, matrix *mass, matrix *Transform, matrix *CoM) {
    rigidBody *body = rigidBody_alloc();
    //todo should I memcpy mass and CoM since they are not going to change?
    body->name = name;
    //memcpy(body->mass, mass, sizeof(matrix));
    copyMatrix(mass, body->mass);
    //memcpy(body->Transform, Transform, sizeof(matrix));
    copyMatrix(Transform, body->Transform);
    //memcpy(body->CoM, CoM, sizeof(matrix));
    copyMatrix(CoM, body->CoM);
    return body;
}

flexBody *flexBody_alloc(int N){
    flexBody *body = (flexBody *) malloc(sizeof(flexBody));
    body->name = malloc(100);
    body->mass = matrix_new(6,6);
    body->transform = matrix_new(6,1);
    body->stiff = matrix_new(6,6);
    body->damping = matrix_new(6,6);
    body->F_0 = matrix_new(6,1);
    body->CoM = matrix_new(6,1);
    body->eta_prev = matrix_new(6,N);
    body->eta_pprev = matrix_new(6,N);
    body->f_prev = matrix_new(6,N);
    body->f_pprev = matrix_new(6,N);
    return body;

}
flexBody *newFlexBody(char *name, matrix *mass, matrix *stiff, matrix *damping, matrix *F_0, int N, double L){
    flexBody *body = flexBody_alloc(N);
    body->name = name;
    copyMatrix(mass, body->mass);
    copyMatrix(stiff, body->stiff);
    copyMatrix(damping, body->damping);
    copyMatrix(F_0, body->F_0);
    body->N = N;
    body->L = L;
    //zeroMatrix(body->CoM);
    //body->CoM = zeros(6,1);
    //zeroMatrix(body->eta_prev);
    //body->eta_prev = zeros(6,N);
    //zeroMatrix(body->eta_pprev);
    //zeroMatrix(body->f_prev);
    //zeroMatrix(body->f_pprev);
    return body;
}
void freeRigidBody(rigidBody *body){
    matrix_free(body->mass);
    //free(body->mass);
    matrix_free(body->Transform);
    matrix_free(body->CoM);
    //free(body->name);//todo do I need to free this?
    free(body);
}
void freeFlexBody(flexBody *body){
    matrix_free(body->mass);
    matrix_free(body->transform);
    matrix_free(body->stiff);
    matrix_free(body->damping);
    matrix_free(body->F_0);
    matrix_free(body->CoM);
    matrix_free(body->eta_prev);
    matrix_free(body->eta_pprev);
    matrix_free(body->f_prev);
    matrix_free(body->f_pprev);
    free(body);
}
void freeJoint(rigidJoint *joint){
    //free(joint->name);
    matrix_free(joint->twistR6);
    free(joint->limits);
    //free(joint->parent);
    //free(joint->)
    free(joint);
}

void robotFree(Robot *robot){
    for(int i = 0; i < robot->numObjects; i++){
        if(robot->objects[i]->type == 0){
            freeRigidBody(robot->objects[i]->object->rigid);
            free(robot->objects[i]->object);
            free(robot->objects[i]);

        }else if(robot->objects[i]->type == 1){
            freeFlexBody(robot->objects[i]->object->flex);
            free(robot->objects[i]->object);
            free(robot->objects[i]);

        }else if(robot->objects[i]->type == 2){
            freeJoint(robot->objects[i]->object->joint);
            free(robot->objects[i]->object);
            free(robot->objects[i]);
        }

    }
    free(robot);

}

rigidJoint *rigidJoint_alloc(){
    rigidJoint *joint = (rigidJoint *) malloc(sizeof(rigidJoint));
    joint->name = malloc(100);
    joint->twistR6 = matrix_new(6,1);
    joint->limits = malloc(sizeof(double) * 2);
    //joint->parent = malloc(sizeof(struct object_s));
    //joint->child = malloc(sizeof(struct object_s));
    return joint;
}

rigidJoint *newRigidJoint(char *name, matrix *twistR6, double position, double velocity, double acceleration, double *limits,
                          double homepos, Object *parent, Object *child) {
    rigidJoint *joint = rigidJoint_alloc();
    joint->name = name;
    //joint->twistR6 = twistR6;

    copyMatrix(twistR6, joint->twistR6);
    joint->position = position;
    joint->velocity = velocity;
    joint->acceleration = acceleration;
    //joint->limits = limits;
    memcpy(joint->limits, limits, sizeof(double) * 2);
    joint->homepos = homepos;

    //todo this whole body union thing is a mess it needs to be redone
    if(parent == NULL) {
        joint->parent = NULL;
    }
    else if(parent->type == 0) {

        joint->parent = malloc(sizeof(struct body_s));
        joint->parent->body = malloc(sizeof(union object_u));
        joint->parent->body->rigid =  parent->object->rigid;
        joint->parent->type = 0;

    }else if(parent->type == 1){

        joint->parent = malloc(sizeof(struct object_s));
        joint->parent->body = malloc(sizeof(union object_u));
        joint->parent->body->flex = parent->object->flex;
        joint->parent->type = 1;

    }

    if(child == NULL) {
        joint->child = NULL;
    }
    else if(child->type == 0) {
        //joint->child->type = 0;
        joint->child = malloc(sizeof(struct object_s));
        joint->child->body = malloc(sizeof(union object_u));
        joint->child->body->rigid = child->object->rigid;
        joint->child->type = 0;

    }else if(child->type == 1){

        joint->child = malloc(sizeof(struct object_s));
        joint->child->body = malloc(sizeof(union object_u));
        joint->child->body->flex = child->object->flex;//todo not sure if this casting will work
        joint->child->type = 1;
    }



    return joint;
}


matrix *plotRobotConfig(Robot *robot, matrix *theta, double numStep) {

    matrix *POS = zeros(3,90);//todo this is a hack, I need to make this dynamic
    matrix *g = matrix_new(4,4);
    eye(g);

    int iii = 0;//num points plotted
    //int i_R = 1;
    matrix *temp6n1 = matrix_new(6,1);
    matrix *temp4x4n1 = matrix_new(4,4);
    union object_u *currObj;
    for(int i = 0; i < (robot->numObjects - 2)/2; i++){
        currObj =  robot->objects[(i*2)+1]->object;
        //assert(robot->objects[(i*2)+2]->type == 0 || robot->objects[(i*2)+2]->type == 1);
        if(currObj->joint->child->type == 0){

            matMult(g, expm_SE3(hat_R6( matrix_scalar_mul(currObj->joint->twistR6, (currObj->joint->homepos + theta->data[(i * theta->numCols)]), temp6n1), temp4x4n1), temp4x4n1), g);

            //setSection(POS, 0,2, iii, iii, getSection(g, 0, 2, 3, 3));
            getSetSection(g, POS, 0, 2, 3, 3, 0, 2, iii, iii);
            iii++;


            if(currObj->joint->child->type == 0) {
                matMult(g, expm_SE3(hat_R6(currObj->joint->child->body->rigid->Transform, temp4x4n1), temp4x4n1), g);
            }else if(currObj->joint->child->type == 1){
                matMult(g, expm_SE3(hat_R6(currObj->joint->child->body->flex->transform, temp4x4n1), temp4x4n1),g);
            }
            //g = matMult(g, expm_SE3(hat_R6(currObj->joint->child->body->rigid->Transform)));

            //setSection(POS, 0,2, iii, iii, getSection(g, 0, 2, 3, 3));
            getSetSection(g, POS, 0, 2, 3, 3, 0, 2, iii, iii);
            iii++;

       }
        else if(currObj->joint->child->type == 1){


            matMult(g, expm_SE3(hat_R6( matrix_scalar_mul(currObj->joint->twistR6, (currObj->joint->homepos + theta->data[(i * theta->numCols)]), temp6n1), temp4x4n1), temp4x4n1), g);

            double ds = currObj->joint->child->body->flex->L / currObj->joint->child->body->flex->N;

            for(int j = 0; j < currObj->joint->child->body->flex->N * numStep; j++){
                int index = ceil(j/numStep);
                if(currObj->joint->child->type )
                temp6n1 = getSection(currObj->joint->child->body->flex->f_prev, 0, 5, index, index, temp6n1);
                expm_SE3(hat_R6(matrix_scalar_mul(temp6n1, ds/numStep, temp6n1), temp4x4n1), temp4x4n1);
                g = matMult(g, temp4x4n1, g);
                getSetSection(g, POS, 0, 2, 3, 3, 0, 2, iii, iii);
                iii +=1;

            }
        }
    }

    //free(currObj);
    matrix_free(g);

    matrix_free(temp6n1);
    matrix_free(temp4x4n1);

    return POS;

}

//matrix *linear_intrpl(matrix **y, float mu){
//
//    return matrix_scalar_mul(matrix_add(y[0], matrix_sub(y[1], y[0])), mu);
//}

//Cosserat Model for a Continuum Semi-Discretized from PDE into a Spatial ODE (!! FLEXIBLE ONLY !!) TODO maybe move this to  FDM
/*
 * y = 12x1    Strain & Velocity Twists (Finite Difference Approximation using previous values)
 * y_h = 12x1  Strain Spatial Rate Twist (Finite Difference Approximation using previous values)
 * f_sh = 6x1  Strain Spatial Rate Twist (Finite Difference Approximation using previous values)
 */
//matrix *COSS_ODE_Dsc(matrix *y, matrix *y_h, matrix *f_sh, flexBody *Body, double c0, matrix *F_dst){
//
//    matrix *f = getSection(y, 0, 5, 0, Body->N - 1);
//    matrix *eta = getSection(y, 6,11, 0, Body->N - 1);
//
//    matrix *f_h = getSection(y_h, 0, 5, 0, Body->N - 1);
//    matrix *eta_h = getSection(y_h, 6,11, 0, Body->N - 1);
//
//
//    matrix *f_t = matrix_add(f, matrix_scalar_mul(f_sh, c0));
//    matrix *eta_t = matrix_add(eta, matrix_scalar_mul(eta_h, c0));
//
////    matrix *f_s = matrix_add(f, matrix_scalar_mul(Body->damping, c0)) +
////            matrix_sub(matrix_add((matMult(Body->mass, eta_t),
////            matrix_sub((matMult(matMult(matrix_transpose(adj_R6(f)),Body->mass), eta)),
////                       matrix_add(matMult(Body->damping, f_sh),
////            matMult(matrix_transpose(adj_R6(f)), (matrix_add(matMult(Body->stiff, matrix_sub(f,Body->F_0->T)), matMult(Body->damping, f_t)) ) ), F_dst)))));
//}



/*
 * function [f_s,eta_s] = Coss_ODE(eta,f,eta_h,f_h,f_sh,K,C,M,c0,f_0,Fd_ext)
 * Cosserat Model for a Continuum Semi-Discretized from PDE into a Spatial ODE (!! FLEXIBLE ONLY !!)
 *
 * DETERMINE:    Spatial Derivative of Velocity and Strain Twists
 * GIVEN:        Velocity, Strain Twists & Stiffness & Mass & Free Strain & Applied Loading
 *
 *     - eta(_h):    Velocity Twists (Finite Difference Approximation using previous values)
 *     - f(_h):      Strain Twists   (Finite Difference Approximation using previous values)
 *     - eta_s:      Velocity Spatial Rate Twists
 *     - f_s:        Strain Spatial Rate Twists
 *     - f_sh:       Strain Spatial Rate Twist (Finite Difference Approximation using previous values)
 *     - f_0:        Free Strain of Body under no Applied Loads
 *     - c_0:        FDM Coefficient for Time Discretization
 *     - K:          Stiffness Matrix of Body in Local BCF
 *     - M:          Mass Matrix of Body in Local BCF
 *     - Fd_ext:     Applied Distributed Load over body expressed in Local BCF
 *
 * Written by BD Bhai

 */
COSS_ODE_OUT *COSS_ODE(matrix *eta, matrix *f, matrix *eta_h, matrix *f_h, matrix *f_sh, matrix *K, matrix *C, matrix *M, double c0, matrix *f_0, matrix *Fd_ext, COSS_ODE_OUT *result) {
//    if(isnan(eta->data[0][0])){
//        printf("eta is nan\n");
//        assert(0);
//    }


    matrix *f_t = matrix_new(6,1);
    matrix_add(matrix_scalar_mul(f, c0, f_t), f_h, f_t);     // Local Time Discretization for history in Local Coordinates

    matrix *eta_t = matrix_new(6,1);
    matrix_add(matrix_scalar_mul(eta, c0, eta_t), eta_h, eta_t);   // Local Time Discretization for history in Local Coordinates


    //holy shit, this is ugly, hope it works, update, it doesn't




    matrix_scalar_mul(C, c0, result->temp6x6n2);
    matrix_add(result->temp6x6n2, K, result->temp6x6n1);
    //66n2 available
    //temp6x6n1 = (K + C*c0)

    matMult(M, eta_t, result->tempR6n1);
    //tempR6n1 = M*eta_t


    matrix_transpose(adj_R6(eta, result->temp6x6n2), result->temp6x6n2);
    //temp6x6n2 = (adj(eta)')
    matMult(M, eta, result->tempR6n3);
    //tempR6n3 = M*eta
    matMult(result->temp6x6n2, result->tempR6n3, result->tempR6n2);
    //tempR6n2 = (adj(eta)')*M*eta
    //tempR6n3 is available

    matMult(C, f_sh, result->tempR6n3);
    //tempR6n3 = C*f_sh

    matrix_sub(result->tempR6n1, result->tempR6n2, result->tempR6n2);
    matrix_sub(result->tempR6n2, result->tempR6n3, result->tempR6n1);
    //tempR6n1 = M*eta_t - (adj(eta)')*M*eta - C*f_sh
    //tempR6n3 is available
    //tempR6n2 is available

    //---------second half of bracket----------------------
    //(adj(f)')*(K*(f - f_0) + C*f_t) + Fd_ext)

    matrix_transpose(adj_R6(f, result->temp6x6n2), result->temp6x6n2);
    //temp6x6n2 = (adj(f)')
    matrix_sub(f, f_0, result->tempR6n3);
    //tempR6n3 = f - f_0

    matMult(K, result->tempR6n3, result->tempR6n3);
    //tempR6n3 = K*(f - f_0)

    matMult(C, f_t, result->tempR6n2);
    //tempR6n2 = C*f_t

    matrix_add(result->tempR6n3, result->tempR6n2, result->tempR6n2);
    //tempR6n2 = K*(f - f_0) + C*f_t

    matMult(result->temp6x6n2, result->tempR6n2, result->tempR6n3);
    //tempR6n3 = (adj(f)')*(K*(f - f_0)+ C*f_t)

    matrix_add(result->tempR6n3, Fd_ext, result->tempR6n3);
    //tempR6n3 = (adj(f)')*(K*(f - f_0)+ C*f_t) + Fd_ext

    matrix_add(result->tempR6n1, result->tempR6n3, result->tempR6n1);
    //tempR6n1 = M*eta_t - (adj(eta)')*M*eta - C*f_sh + (adj(f)')*(K*(f - f_0)+ C*f_t) + Fd_ext

    matrix_solve(result->temp6x6n1, result->tempR6n1, result->f_s);


    matrix_add(f_t, matMult(adj_R6(eta, result->temp6x6n1), f, result->tempR6n1), result->eta_s);
//    if(isnan(result->eta_s->data[1][0]) || isnan(result->eta_s->data[0][0])){
//        printf("eta is nan\n");
//        assert(0);
//    }

    matrix_free(eta_t);

    matrix_free(f_t);
    //printMatrix(adj_R6(f));
//    printMatrix(result->f_s);
//    printMatrix(result->eta_s);
    return result;
}

char* objToJson(Object *obj){
    char *result = malloc(20000);//todo set this to whatever the upper limit is for bodies
    if(obj->type == 0){//rigid
        sprintf(result, "%s{\n\"Name\": \"%s\",\n\"Type\": \"RIGID\",",result, obj->object->rigid->name);
        sprintf(result, "%s\n\"Mass\": %s,",result, matrixToJson(obj->object->rigid->mass, "matlab"));
        sprintf(result, "%s\n\"Transform\": %s,",result, matrixToJson(obj->object->rigid->Transform, "matlab"));
        sprintf(result, "%s\n\"CoM\": %s}",result, matrixToJson(obj->object->rigid->CoM, "matlab"));
    }else if(obj->type == 1){//flex
        sprintf(result, "%s{\n\"Name\": \"%s\",\n\"Type\": \"FLEXIBLE\",",result, obj->object->flex->name);
        sprintf(result, "%s\n\"Mass\": %s,",result, matrixToJson(obj->object->flex->mass, "matlab"));//todo is this a memory leak?
        sprintf(result, "%s\n\"Transform\": %s,",result, matrixToJson(obj->object->flex->transform, "matlab"));
        sprintf(result, "%s\n\"Stiff\": %s,",result, matrixToJson(obj->object->flex->stiff, "matlab"));
        sprintf(result, "%s\n\"Damp\": %s,",result, matrixToJson(obj->object->flex->damping, "matlab"));
        sprintf(result, "%s\n\"CoM\": %s,",result, matrixToJson(obj->object->flex->CoM, "matlab"));
        sprintf(result, "%s\n\"F_0\": %s,",result, matrixToJson(obj->object->flex->F_0, "matlab"));
        sprintf(result, "%s\n\"N\": %d,",result, obj->object->flex->N);
        sprintf(result, "%s\n\"L\": %.18f,",result, obj->object->flex->L);
        sprintf(result, "%s\n\"eta_prev\": %s,",result, matrixToJson(obj->object->flex->eta_prev, "matlab"));
        sprintf(result, "%s\n\"eta_pprev\": %s,",result, matrixToJson(obj->object->flex->eta_pprev, "matlab"));
        sprintf(result, "%s\n\"f_prev\": %s,",result, matrixToJson(obj->object->flex->f_prev, "matlab"));
        sprintf(result, "%s\n\"f_pprev\": %s}",result, matrixToJson(obj->object->flex->f_pprev, "matlab"));
    }else if(obj->type == 2){//jointe
        sprintf(result, "%s{\n\"Name\": \"%s\",\n\"Type\": \"RIGID\",",result, obj->object->joint->name);
        char *temp = matrixToJson(obj->object->joint->twistR6, "matlab");
        sprintf(result, "%s\n\"Twist\": %s,", result, temp);

        sprintf(result, "%s\n\"Position\": %.18f,",result, obj->object->joint->position);
        sprintf(result, "%s\n\"Vel\": %.18f,",result, obj->object->joint->velocity);
        sprintf(result, "%s\n\"Accel\": %.18f,",result, obj->object->joint->acceleration);
        sprintf(result, "%s\n\"Limit\": [%.18f,%.18f],",result, obj->object->joint->limits[0], obj->object->joint->limits[1]);
        sprintf(result, "%s\n\"HomePos\": %.18f,",result, obj->object->joint->homepos);


        //todo parent and child in joint should be changed to object with assert
        Object *parent = malloc(sizeof(Object));
        parent->object = malloc(sizeof(union object_u));
        if(obj->object->joint->parent->type == 0) {
            parent->type = 0;
            parent->object->rigid = obj->object->joint->parent->body->rigid;
        }else if(obj->object->joint->parent->type == 1){
            parent->type = 1;
            parent->object->flex = obj->object->joint->parent->body->flex;
        }else{
            printf("something is very wrong with the setup got non-body as parent");
        }

        Object *child = malloc(sizeof(Object));
        child->object = malloc(sizeof(union object_u));
        if(obj->object->joint->child->type == 0) {
            child->type = 0;
            child->object->rigid = obj->object->joint->child->body->rigid;
        }else if(obj->object->joint->child->type == 1){
            child->type = 1;
            child->object->flex = obj->object->joint->child->body->flex;
        }else{
            printf("something is very wrong with the setup got non-body as child");
        }
        sprintf(result, "%s\n\"Parent\": %s,",result, objToJson(parent));
        sprintf(result, "%s\n\"Child\": %s}",result, objToJson(child));
    }

    sprintf(result, "%s\n", result);


    return result;

}

void robotToFile(Robot *robot, char *filename){



    FILE *f = fopen(filename, "a");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }

    fprintf(f, "[");
    for(int i = 0; i<robot->numObjects; i++) {
            printf("Writing object %d\n", i);
            printf("Object type: %d\n", robot->objects[i]->type);
            printf("Object name: %s\n", robot->objects[i]->object->rigid->name);
            char *temp = objToJson(robot->objects[i]);
            fprintf(f, "%s\n", temp);
            fprintf(f,",");
            fflush(f);
            free(temp);
    }
    fprintf(f, "]");

    fclose(f);

}
/* save solver info to csv
 *
 *
*/
void solverSave(char *solver, char *filename, int code, int iters, double residual, int fwd){
    FILE *file = fopen(filename, "a");
    if (file == NULL) {
        fprintf(stderr, "Error opening file: %s\n", filename);
        return;
    }



    // Write the solver info
    fprintf(file, "%s,%s,%d,%d,%d", solver, filename, code, iters, fwd);

    // Write the residuals

    fprintf(file, ",%.20f", residual);


    // New line for the next entry
    fprintf(file, "\n");

    // Close the file
    fclose(file);
}


void addRobotState(Robot *robot, char* filename, int num){
    FILE *f = fopen(filename, "a");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }
    fprintf(f, ",\n\"%d\":", num);
    fclose(f);
    robotToFile(robot, filename);


}


rigidKin *rigidKinAlloc(){
    rigidKin *kin = malloc(sizeof(rigidKin));
    kin->g_cur = zeros(4,4);
    kin->g_act_wrt_prev = zeros(4,4);
    kin->eta = zeros(6,1);
    kin->d_eta = zeros(6,1);
    return kin;
}

void freeRigidKin(rigidKin *kin){
    matrix_free(kin->g_cur);
    matrix_free(kin->g_act_wrt_prev);
    matrix_free(kin->eta);
    matrix_free(kin->d_eta);
    free(kin);
}
flexDyn *flexDynAlloc(){
    flexDyn *dyn = malloc(sizeof(flexDyn));
    dyn->d_eta_end = zeros(6,1);
    dyn->g_end = zeros(4,4);
    dyn->eta = NULL;
    dyn->f = NULL;

    return dyn;
}

void freeFlexDyn(flexDyn *dyn){
    matrix_free(dyn->f);
    matrix_free(dyn->eta);
    matrix_free(dyn->g_end);
    matrix_free(dyn->d_eta_end);
    free(dyn);
}

COSS_ODE_OUT *odeAlloc(){
    COSS_ODE_OUT *out = malloc(sizeof(COSS_ODE_OUT));
    out->eta_s = zeros(6,1);
    out->f_s = zeros(6,1);

    out->temp6x6n1 = matrix_new(6,6);
    out->temp6x6n2 = matrix_new(6,6);

    out->tempR6n1 = matrix_new(6,1);
    out->tempR6n2 = matrix_new(6,1);
    out->tempR6n3 = matrix_new(6,1);
    return out;
}
void freeCOSS_ODE_OUT(COSS_ODE_OUT *out){
    if(out != NULL) {
        matrix_free(out->eta_s);
        matrix_free(out->f_s);

        matrix_free(out->temp6x6n1);
        matrix_free(out->temp6x6n2);

        matrix_free(out->tempR6n1);
        matrix_free(out->tempR6n2);
        matrix_free(out->tempR6n3);
        free(out);
    }
}

flexDyn *flex_dyn(matrix *g_base, matrix *F_dist, matrix *F_base, flexBody *body, matrix *eta_base, double c0, double c1, double c2, flexDyn *result){
//    printf("Eta_base");
//    printMatrix(eta_base);

    //matrix *eta = zeros(6, body->N);
    //matrix *f = zeros(6, body->N);

    zeroMatrix(result->eta);
    zeroMatrix(result->f);
    //result->eta = zeros(6, body->N);
    //result->f = zeros(6, body->N);
    //assert(hasNan(F_base) == 0);
    matrix **g = (matrix **)malloc(sizeof(matrix *) * body->N);

    matrix *temp6xNn1 = matrix_new(6, body->N);
    matrix *temp6xNn2 = matrix_new(6, body->N);


    matrix *eta_h = matrix_new(6, body->N);
    matrix_add(matrix_scalar_mul(body->eta_prev, c1, temp6xNn1), matrix_scalar_mul(body->eta_pprev, c2, temp6xNn2), eta_h);

    matrix *f_h = matrix_new(6, body->N);
    matrix_add(matrix_scalar_mul(body->f_prev, c1, temp6xNn1), matrix_scalar_mul(body->f_pprev, c2, temp6xNn2), f_h);

    //set g to <4, 4, numBodies>
    for(int i = 0; i < body->N; i++){
        g[i] = zeros(4,4);
    }

    matrix *tempR6n1 = matrix_new(6,1);
    matrix *tempR6n2 = matrix_new(6,1);
    matrix *tempR6n3 = matrix_new(6,1);
    matrix *tempR6n4 = matrix_new(6,1);
    matrix *tempR6n5 = matrix_new(6,1);

    matrix *temp4x4n1 = matrix_new(4,4);
    //assert(hasNan(result->f) == 0);
    setSection(result->f, 0, 5, 0, 0, matrix_add(matrix_solve(body->stiff,F_base, tempR6n1), body->F_0, tempR6n1));
    //assert(hasNan(result->f) == 0);

    setSection(result->eta, 0, 5, 0, 0, eta_base);
    //memcpy(g[0], g_base, sizeof(matrix));
    copyMatrix(g_base, g[0]);
    //g[0] = g_base;
    double ds = ((double) body->L) / (body->N - 1);


    matrix *f_sh = matrix_new(6,1);
    COSS_ODE_OUT *ode = odeAlloc();

    //assert(hasNan(result->f) == 0);
    for(int i = 0; i < body->N-1; i++) {

        matrix_scalar_mul(matrix_sub(getSection(body->f_prev, 0,5,i+1,i+1, tempR6n1), getSection(body->f_prev, 0,5,i,i, tempR6n2),tempR6n1),c1, tempR6n1);
        matrix_scalar_mul(matrix_sub(getSection(body->f_pprev, 0,5,i+1,i+1, tempR6n2), getSection(body->f_pprev, 0,5,i,i, tempR6n3), tempR6n2),c2, tempR6n2);
        matrix_add(tempR6n1,tempR6n2,tempR6n1);

        elemDiv(
            tempR6n1,
                ds,
                f_sh
        );

        //[f_s,eta_s] = Coss_ODE(eta(:,i), f(:,i), eta_h(:,i), f_h(:,i), f_sh,BODY.Stiff,BODY.Damp,BODY.Mass,c0,BODY.F_0,F_dist(:,i));

        COSS_ODE(getSection(result->eta,0,5,i,i, tempR6n1), getSection(result->f,0,5,i,i, tempR6n2),
                        getSection(eta_h,0,5,i,i, tempR6n3), getSection(f_h, 0,5,i,i, tempR6n4),
                        f_sh, body->stiff, body->damping, body->mass, c0, body->F_0,
                        getSection(F_dist, 0,5,i,i, tempR6n5), ode);
        //assert(hasNan(g[i+1])==0);
        /*
         * f(:,i+1) = f(:,i) + f_s*ds;                     %[se(3)]    Assuming Euler Integration
         * eta(:,i+1) = eta(:,i) + eta_s*ds;               %[se(3)]    Assuming Euler Integration
         * g(:,:,i+1) = g(:,:,i) * expm(hat(f(:,i))*ds);   %[SE(3)]    Assuming Lie-Euler Geometric Integration
         */
        //assert(hasNan(result->f) == 0);
        setSection(result->f,0,5,i+1,i+1,
                   (matrix_add(getSection(result->f,0,5,i,i, tempR6n1), matrix_scalar_mul(ode->f_s,ds, tempR6n2), tempR6n1)));
        //assert(hasNan(g[i+1])==0);

        setSection(result->eta,0,5,i+1,i+1,
                   (matrix_add(getSection(result->eta,0,5,i,i, tempR6n1), matrix_scalar_mul(ode->eta_s,ds, tempR6n2), tempR6n1))
        );

        getSection(result->f,0,5,i,i, tempR6n1);

        hat_R6(tempR6n1, temp4x4n1);

        matrix_scalar_mul(temp4x4n1, ds, temp4x4n1);

        expm(temp4x4n1, temp4x4n1);
        matMult(g[i], temp4x4n1,g[i+1]);
    }

    freeCOSS_ODE_OUT(ode);


    copyMatrix(g[body->N-1], result->g_end);


    matrix_add(
            matrix_scalar_mul(getSection(result->eta, 0,5,result->eta->numCols - 1, result->eta->numCols -1, tempR6n1), c0, tempR6n1),
            getSection(eta_h, 0,5,eta_h->numCols-1, eta_h->numCols-1, tempR6n2),
            result->d_eta_end);
    //matrix_free(result->f)
    //result->f = f;
    //result->eta = eta;

    //freeCOSS_ODE_OUT(&ode);


    //todo there has to be a better way to do this
    for(int i = 0; i < body->N; i++){
        matrix_free(g[i]);
    }
    matrix_free(temp6xNn1);
    matrix_free(temp6xNn2);
    matrix_free(tempR6n1);
    matrix_free(tempR6n2);
    matrix_free(tempR6n3);
    matrix_free(tempR6n4);
    matrix_free(tempR6n5);
    matrix_free(temp4x4n1);
    matrix_free(f_h);
    matrix_free(eta_h);
    matrix_free(f_sh);

    //matrix_free(eta);
    return result;
}






//return first flexible body or -1 if none
int getBCStart(Robot *robot){
    //int first = robot->numObjects +3;
    for(int i = 1; i<(robot->numObjects-3)/2; i++){
        if(robot->objects[2*i]->type == 1){
            //first = i;
            return i;
        }
    }
    return -1;
}

//return last flexible body or -1 if none
int getBCEnd(Robot *robot){
    int last = -1;
    for(int i = 1; i < (robot->numObjects-3)/2; i++){
        if(robot->objects[2*i]->type == 1 && i > last){
            last = i;

        }
    }
    return last;

}


/*  Flex-Rigid Multi-Body Open-Chain Manipulator Boundary Condition Solver to compute the
 * difference between the constraint forces implied at the end of the last flexible body
 * based on the assumption of constraint force at the beginning of the first flexible body
 * compared to that implied by the external force applied at the EE.
 *
 * DETERMINE:    Error in the Constraint Force at end of Last Flexible Body
 * GIVEN:        Manipulator Definition & Joint Position, Velocity, Acceleration &
 *               Applied Loading & FDM Discretization Coeff.
 *
 *     - InitGuess:      Guess for the Strain at the Beginning of the First Elastic Element (guessed / iterated)
 *     - ROBOT:          Definition of Manipulator (Collection of Bodies and Joints)
 *     - THETA:          Actuated Joint Positions
 *     - THETA_DOT:      Actuated Joint Velocities
 *     - THETA_DDOT:     Actuated Joint Accelerations
 *     - F_ext:          Applied Load at the EE expressed in EE BCF
 *     - c0:             FDM Coeff - Current Time Step
 *     - c1:             FDM Coeff - Previous Time Step
 *     - c2:             FDM Coeff - PrePrevious Time Step
 *     - RRC:            Robot Reference Configuration
 *     - RAC:            Robot Actuated Configuration
 *     - BCF:            Body Coordinate Frame
 *
 * Written by BD Bhai
 *
 *  robot:       Robot Definition
 * Theta:       Joint Position                          <1xnumJoints>
 * Theta_dot:   Joint Velocity                          <1xnumJoints>
 * Theta_DDot:  Joint Acceleration                      <1xnumJoints>
 * F_ext:       External Wrench                         todo <6x1> right?
 * dt:          Time Step                               double
 * InitGuess:   Initial Guess for the Solution          todo <6x1> right?
 * c0 :         FDM Coeff - Current Time Step            double
 * c1 :         FDM Coeff - Previous Time Step           double
 * c2 :         FDM Coeff - PrePrevious Time Step        double
 *
*/

matrix *Flex_MB_BCS(matrix *InitGuess, Flex_MB_BCS_params *params){


    //assert(hasNan(InitGuess) == 0);
    Robot *robot = params->robot;


    matrix *F_ext = params->F_ext;
    double c0 = params->c0;
    double c1 = params->c1;
    double c2 = params->c2;
    double dt = params->dt;
    matrix *theta = params->Theta;
    matrix *theta_dot = params->Theta_dot;
    matrix *theta_ddot = params->Theta_ddot;
    matrix *C_des = params->C_des;
    matrix *F_0 = params->F_0;
    int Inv = params->inv;


    int BC_Start = getBCStart(robot);
    int BC_End = getBCEnd(robot);
    int numBody = (robot->numObjects-3)/2;

    if(BC_Start == -1){
        //todo not sure what to do here, it might just work?
        //printf("No Flexible Body Found (Flex_MB_BCS)\n");
        return zeros(6,1);
    }

    //todo this is the same as in IDM_MB_RE, it might be faster to pass all these as arguments or maybe a struct or something
    matrix **g_ref =  malloc(sizeof(matrix) * (numBody+2));           //[SE(3) X N+2]  Transformation to i_th C-BCF from/in base BCF for RRC
    for(int i = 0; i < numBody+2; i++){
        g_ref[i] = zeros(4,4);
    }
    matrix **g_act_wrt_prev = malloc(sizeof(matrix) * (numBody+2));  //[SE(3) X N+2]  Transformation to i-1_th C-BCF from/in i_th BCF for RAC
    for(int i = 0; i < numBody+2; i++){
        g_act_wrt_prev[i] = zeros(4,4);
    }

    matrix *eta = zeros(6,7);      //todo magic number         //[se(3) X N+2]  Twists for each BCF + Base + EE in BCF
    matrix *d_eta = zeros(6,7);        //todo magic number     //[se(3) X N+2]  Twist Rate for each BCF + Base + EE Frame in BCF
    matrix *F = zeros(6,6);   //todo magic number              //[se(3) X N+1]  Wrench for each Joint + EE in BCF

    // Set Initial Conditions

    eye(g_ref[0]);           //[SE(3)]    Base Frame Located @ Base Frame so Identity Transform
    eye(g_act_wrt_prev[0]);  //[SE(3)]    Base Frame Located @ Base Frame so Identity Transform

    matrix *F_temp = zeros(6,1);

    //recursive definition of dynamics using Euler-pointcare EOM
    Object *curr_joint;
    Object *curr_body;
    matrix *CoM2CoM = zeros(4,4);
    matrix *F_dist = NULL;
    //todo it is probably faster to have these as preallocated globals or something so they dont have to be allocated and deallocated every iteration in every function
    matrix *tempR6n1 = matrix_new(6,1);
    matrix *tempR6n2 = matrix_new(6,1);
    matrix *tempR6n3 = matrix_new(6,1);

    matrix *temp6x6n1 = matrix_new(6,6);
    matrix *temp6x6n2 = matrix_new(6,6);

    rigidKin *kin = rigidKinAlloc();
    flexDyn *dyn = flexDynAlloc();

    //matrix *parentCoM ;
    //matrix *childCoM;
    for(int i = 1; i <= BC_End; i++){

        //printMatrix(&F_temp);
        //printf("\n\n");

        curr_joint = robot->objects[2 * (i - 1)+1];
        curr_body = robot->objects[2 * i ];

        //printMatrix(eta);
        CoM2CoM = getCoM2CoM(curr_joint->object->joint, CoM2CoM);

        //todo double check matrix sizes



        actuateRigidJoint(g_ref[i-1], CoM2CoM, curr_joint->object->joint, getSection(eta, 0,5,i-1,i-1, tempR6n1), getSection(d_eta, 0,5,i-1,i-1, tempR6n2), kin);
        //memcpy(g_ref[i], kin->g_cur, sizeof(matrix));
        //g_ref[i] = kin->g_cur;
        copyMatrix(kin->g_cur, g_ref[i]);

        //memcpy(g_act_wrt_prev[i], kin->g_act_wrt_prev, sizeof(matrix));
        //g_act_wrt_prev[i] = kin->g_act_wrt_prev;
        copyMatrix(kin->g_act_wrt_prev, g_act_wrt_prev[i]);

        //assert(hasNan(eta) == 0);
        setSection(eta, 0,5,i,i, kin->eta);
        //assert(hasNan(eta) == 0);
        setSection(d_eta, 0,5,i,i, kin->d_eta);


        matMult(matrix_transpose(adj(g_act_wrt_prev[i], temp6x6n1), temp6x6n2), F_temp, F_temp);//why is this not a pointer
        //assert(hasNan(g_ref[i])==0);
        if(curr_body->type == 1) {//flexible body
            if(dyn->eta != NULL){
                matrix_free(dyn->eta);
            }
            dyn->eta = zeros(6, curr_body->object->flex->N);

            if(dyn->f != NULL){
                matrix_free(dyn->f);
            }

            dyn->f = zeros(6, curr_body->object->flex->N);

            if(i == BC_Start ) {
                //ROBOT{2*i-1}.Stiff * (InitGuess - ROBOT{2*i-1}.F_0);
                //assert(hasNan(F) == 0);
                setSection(F, 0, 5, i, i, matMult(curr_body->object->flex->stiff,
                                                  matrix_sub(InitGuess, curr_body->object->flex->F_0, tempR6n1), tempR6n1));
                //assert(hasNan(F) == 0);
                //printMatrix(F);
            } else{
                //assert(hasNan(F) == 0);
                setSection(F,0,5,i,i, F_temp);

                //assert(hasNan(F) == 0);
            }

            if(F_dist != NULL){
                matrix_free(F_dist);
            }
            F_dist = zeros(6, curr_body->object->flex->N);


            flex_dyn(g_ref[i], F_dist, getSection(F, 0, 5, i, i, tempR6n1), curr_body->object->flex,
                     getSection(eta, 0, 5, i, i, tempR6n2), c0, c1, c2, dyn);

            setSection(d_eta, 0,5,i,i, dyn->d_eta_end);
            //memcpy(g_ref[i], dyn->g_end, sizeof(matrix));
            copyMatrix(dyn->g_end, g_ref[i]);
            //assert(hasNan(g_ref[i]) == 0);
//            g_ref[i] = dyn->g_end;//is this the leak??
            //d_eta[i] = *dyn->d_eta_end;


            matMult(curr_body->object->flex->stiff, matrix_sub(getSection(dyn->f,0,5,dyn->f->numCols-1,dyn->f->numCols-1, tempR6n1), curr_body->object->flex->F_0, tempR6n1), F_temp);
            setSection(eta,0,5,i, i, getSection(dyn->eta,0,5,dyn->eta->numCols-1, dyn->eta->numCols-1, tempR6n1));


        }else if(i>BC_Start) {//rigid bodies
            setSection(F,0,5,i,i, F_temp);// [N;Nm] Save Wrench Between i,i-1_th Body @ CoM Expressed in BCF

            /*
             * F_temp = F(:,i) + transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i)- ROBOT{2*i-1}.Mass*d_eta(:,i);
             */
            getSection(F, 0,5,i,i, tempR6n1);
            getSection(eta,0,5,i,i, tempR6n2);
            adj_R6(tempR6n2, temp6x6n1);
            matrix_transpose(temp6x6n1, temp6x6n1);
            matMult(curr_body->object->rigid->mass, getSection(eta,0,5,i,i, tempR6n3), tempR6n3);
            matMult(temp6x6n1, tempR6n3, tempR6n2);
            matrix_add(tempR6n1, tempR6n2, tempR6n1);


            matMult(curr_body->object->rigid->mass, getSection(d_eta,0,5,i,i, tempR6n2), tempR6n3);
            matrix_sub(tempR6n1
                    ,tempR6n3, F_temp);
//            F_temp = *matrix_sub(matrix_add(
//                    getSection(F, 0,5,i,i),
//                    matMult(matrix_transpose(adj_R6(getSection(eta,0,5,i,i))), matMult(curr_body->object->rigid->mass,getSection(eta,0,5,i,i)))
//                    ),matMult(curr_body->object->rigid->mass,getSection(eta,0,5,i,i))
//            );
        }
        //assert(hasNan(g_ref[i])==0);

    }




    // ALGORITHM FOR LAST ELASTIC BODY TO END OF MANIPULATOR FOR BC LOADS

    for(int i = BC_End+1; i < numBody+2; i++){//todo fix magic number
        curr_joint = robot->objects[2 * (i - 1)+1];



        CoM2CoM = getCoM2CoM(curr_joint->object->joint, CoM2CoM);


        //% Use Rigid-Body Kinematic Equations to find Velocities, Accelerations and Transformations
        //        [g_ref(:,:,i),g_act_wrt_prev(:,:,i),eta(:,i),d_eta(:,i)] = Rigid_Kin(g_ref(:,:,i-1), CoM2CoM, ROBOT{2*(i-1)}, eta(:,i-1), d_eta(:,i-1));
        actuateRigidJoint(g_ref[i - 1], CoM2CoM, curr_joint->object->joint, getSection(eta, 0,5,i-1,i-1, tempR6n1), getSection(d_eta, 0,5,i-1,i-1, tempR6n2), kin);

        //g_ref[i] = kin->g_cur;
        copyMatrix(kin->g_cur, g_ref[i]);
        //g_act_wrt_prev[i] = kin->g_act_wrt_prev;//same here as above, is this a leak??
        copyMatrix(kin->g_act_wrt_prev, g_act_wrt_prev[i]);


        setSection(eta, 0,eta->numRows-1,i,i, kin->eta);
        setSection(d_eta, 0,d_eta->numRows-1,i,i, kin->d_eta);


    }

//    for(int i = numBody + 1; i >= 0; i--){
//        matrix_free(g_ref[i]);
//        matrix_free(g_act_wrt_prev[i]);
//    }

    setSection(F, 0,F->numRows-1,F->numCols - 1,F->numCols - 1, F_ext); //TODO this needs to be added back in, it is the applied wrench to the end effector



    matrix *bodyMass = matrix_new(6,6);
    for(int i = BC_End+1; i >= numBody ; i--){



        //    %[]         End of {SECTION III} Algorithm
        curr_body = robot->objects[2 * i ];
        if(curr_body->type == 1){//flex
            //bodyMass = curr_body->object->flex->mass;
            copyMatrix(curr_body->object->flex->mass, bodyMass);

        }else if(curr_body->type == 0){//rigid
            //bodyMass = curr_body->object->rigid->mass;
            copyMatrix(curr_body->object->flex->mass, bodyMass);
        }

        //got distracted halfway through this so it could be wrong (it was)
        setSection(F,0,5,i-1,i-1,
                   matrix_sub(
                   matrix_add(
                   matMult(
                   matrix_transpose(adj(g_act_wrt_prev[i+1], temp6x6n1), temp6x6n1),
                   getSection(F,0,F->numRows-1,i,i, tempR6n1)
                   , tempR6n1),
                   matMult(bodyMass, getSection(d_eta,0,d_eta->numRows-1,i,i, tempR6n2), tempR6n2), tempR6n2),
                   matMult(matMult(matrix_transpose(adj_R6(getSection(eta,0,eta->numRows-1,i,i, tempR6n3), temp6x6n2), temp6x6n2), bodyMass, temp6x6n2), getSection(eta,0,eta->numRows-1,i,i, tempR6n3), tempR6n3),
                   tempR6n1));

        //printMatrix(F);
    }





    matrix *out = matrix_new(6,1);
    matrix_sub(F_temp, getSection(F,0,5,BC_End,BC_End, out), out);
    //free(bodyMass);
    matrix_free(eta);
    //assert(1 == 0);



    //assert(hasNan(out)==0);
    for(int i = 0; i < (numBody+2); i++){
        matrix_free(g_ref[i]);
        matrix_free(g_act_wrt_prev[i]);
    }
    matrix_free(F_dist);
    freeFlexDyn(dyn);
    freeRigidKin(kin);
    matrix_free(F);
    matrix_free(d_eta);
    matrix_free(F_temp);
    matrix_free(CoM2CoM);
    //matrix_free(F_dist);

    matrix_free(tempR6n1);
    matrix_free(tempR6n2);
    matrix_free(tempR6n3);

    matrix_free(temp6x6n1);
    matrix_free(temp6x6n2);
    return out;

}

int F_Flex_MB_BCS(matrix *InitGuess, matrix* result, Flex_MB_BCS_params *params){

    //assert(hasNan(InitGuess) == 0);
    Robot *robot = params->robot;
    if(hasNan(InitGuess)){
        return GSL_EFAILED;
    }
    assert(isnan(robot->objects[1]->object->joint->velocity)==0);
    matrix *F_ext = matrix_new(params->F_ext->numRows, params->F_ext->numCols);
    copyMatrix(params->F_ext, F_ext);

    double c0 = params->c0;
    double c1 = params->c1;
    double c2 = params->c2;
    double dt = params->dt;



    matrix *theta = matrix_new(params->Theta->numRows, params->Theta->numCols);
    copyMatrix(params->Theta, theta);


    matrix *theta_dot = matrix_new(params->Theta_dot->numRows, params->Theta_dot->numCols);
    copyMatrix(params->Theta_dot, theta_dot);


    matrix *theta_ddot = matrix_new(InitGuess->numRows, InitGuess->numCols);
    copyMatrix(InitGuess, theta_ddot);
    matrix *theta_ddot_old = zeros(theta_ddot->numRows, theta_ddot->numCols);
    copyMatrix(params->Theta_ddot, theta_ddot_old);
    copyMatrix(InitGuess, params->Theta_ddot);

    matrix *C_des = matrix_new(params->C_des->numRows, params->C_des->numCols);
    copyMatrix(params->C_des, C_des);


    matrix *F_0 = matrix_new(params->F_0->numRows, params->F_0->numCols);
    copyMatrix(params->F_0, F_0);


    int Inv = params->inv;



    int BC_Start = getBCStart(robot);//todo these dont work
    int BC_End = getBCEnd(robot);
    int numBody = (robot->numObjects-3)/2;



    matrix *str_guess;
    assert(isnan(robot->objects[1]->object->joint->velocity)==0);
    matrix *accel_old = zeros(theta_ddot->numRows,1);
    matrix *vel_old = zeros(theta_dot->numRows,1);
    int num = 0;
    for(int i = 0; i < (robot->numObjects/2)-1; i++){
        if(robot->objects[(2*i)+1]->type == 2){
            accel_old->data[num] = robot->objects[(2*i)+1]->object->joint->acceleration;
            robot->objects[(2*i)+1]->object->joint->acceleration = InitGuess->data[num];

            vel_old->data[num] = robot->objects[(2*i)+1]->object->joint->velocity;
            assert(isnan(robot->objects[(2*i)+1]->object->joint->velocity)==0);
            robot->objects[(2*i)+1]->object->joint->velocity += InitGuess->data[num] * dt;
            num ++;
        }
    }
    str_guess = zeros(F_0->numRows,1);
    //copyMatrix(F_0, str_guess);


    #if VERBOSE >= 2
    printf("\t\t-------------------F_FLEX-----------------------------\n");
    printf("\t\tInitial Guess For Forward boundary condition solver:\n");
    printMatrix(F_0);
    //assert(hasNan(InitGuess) == 0);
    #endif
    //printf("F_FLEX_MB_BCS SOLVER START");

    matrix *tempGuess = zeros(6,1);

    copyMatrix(F_0, tempGuess);
    //assert(isnan(robot->objects[1]->object->joint->velocity)==0);
    params->inv = 0;
    int status = find_roots_hybrid(tempGuess, params, 0, TOLERANCE_INV);
    //assert(isnan(robot->objects[1]->object->joint->velocity)==0);
    if (status != 0) {
        #if VERBOSE >= 2
        printf("\t\thybrid method failed to converge in F_FLEX. Trying levmar\n");
        #endif
        copyMatrix(F_0, tempGuess);
        status = find_roots_levmarqrt(tempGuess, params, 0, TOLERANCE_INV);
        if (status != 6 ) {
            #if VERBOSE >= 2
            printf("\t\tlevmar method failed to converge in F_FLEX trying newton\n");
            #endif
            copyMatrix(F_0, tempGuess);
            status = find_roots_newton(tempGuess, params, 0, TOLERANCE_INV);
            if(status != 0){

                #if VERBOSE >= 2
                printf("\tALL FAILED IN F_FLEX_MB_BCS\n");
                #endif


                #if SOLVER_ERRORS == 1
                    assert(1 == 0);
                #endif
                //return GSL_EFAILED;

            }
        }
    }
    params->inv = 1;
    #if VERBOSE >= 2

    printMatrix(tempGuess);
    printf("-------------------F_FLEX END-----------------------------\n");
    #endif
    copyMatrix(tempGuess, str_guess);
    copyMatrix(theta_ddot_old, params->Theta_ddot);

    matrix *tempT = matrix_new(1, 6);
    matrixToFile(matrix_transpose(str_guess, tempT), "Flex.csv");
    //printf("FUFLEX_MB_BCS SOLVER END\n");
    //printMatrix(str_guess);
    //printf("\n");

    //todo this is the same as in IDM_MB_RE, it might be faster to pass all these as arguments or maybe a struct or something
    matrix **g_ref =  malloc(sizeof(matrix) * (numBody+2));           //[SE(3) X N+2]  Transformation to i_th C-BCF from/in base BCF for RRC
    for(int i = 0; i < numBody+2; i++){
        g_ref[i] = zeros(4,4);
    }
    matrix **g_act_wrt_prev = malloc(sizeof(matrix) * (numBody+2));  //[SE(3) X N+2]  Transformation to i-1_th C-BCF from/in i_th BCF for RAC
    for(int i = 0; i < numBody+2; i++){
        g_act_wrt_prev[i] = zeros(4,4);
    }

    matrix *eta = zeros(6,7);      //todo magic number         //[se(3) X N+2]  Twists for each BCF + Base + EE in BCF
    matrix *d_eta = zeros(6,7);        //todo magic number     //[se(3) X N+2]  Twist Rate for each BCF + Base + EE Frame in BCF
    matrix *F = zeros(6,7);   //todo magic number              //[se(3) X N+1]  Wrench for each Joint + EE in BCF

    matrix *C;
    if(Inv) {
        C = zeros(1, numBody);   //todo magic number
    }
    // Set Initial Conditions

    eye(g_ref[0]);           //[SE(3)]    Base Frame Located @ Base Frame so Identity Transform
    eye(g_act_wrt_prev[0]);  //[SE(3)]    Base Frame Located @ Base Frame so Identity Transform

    matrix *F_temp = zeros(6,1);

    //recursive definition of dynamics using Euler-pointcare EOM
    Object *curr_joint;
    Object *curr_body;
    matrix *CoM2CoM = zeros(4,4);
    matrix *F_dist = NULL;
    //todo it is probably faster to have these as preallocated globals or something so they dont have to be allocated and deallocated every iteration in every function
    matrix *tempR6n1 = matrix_new(6,1);
    matrix *tempR6n2 = matrix_new(6,1);
    matrix *tempR6n3 = matrix_new(6,1);
    matrix *tempR6n4 = matrix_new(6,1);

    matrix *tempR6t = matrix_new(1,6);

    matrix *temp4x4n1 = matrix_new(4,4);

    matrix *tempC6n1 = matrix_new(1,6);
    matrix *temp6x6n1 = matrix_new(6,6);
    matrix *temp6x6n2 = matrix_new(6,6);
    matrix *temp1 = matrix_new(1,1);
    rigidKin *kin = rigidKinAlloc();
    flexDyn *dyn = flexDynAlloc();

    for(int i = 1; i <= numBody+1; i++){

        curr_joint = robot->objects[2 * (i - 1)+1];
        curr_body = robot->objects[2 * i ];


        CoM2CoM = getCoM2CoM(curr_joint->object->joint, CoM2CoM);



        actuateRigidJoint(g_ref[i-1], CoM2CoM, curr_joint->object->joint, getSection(eta, 0,5,i-1,i-1, tempR6n1), getSection(d_eta, 0,5,i-1,i-1, tempR6n2), kin);


        copyMatrix(kin->g_cur, g_ref[i]);

        copyMatrix(kin->g_act_wrt_prev, g_act_wrt_prev[i]);

        setSection(eta, 0,5,i,i, kin->eta);
        setSection(d_eta, 0,5,i,i, kin->d_eta);


        matMult(matrix_transpose(adj(g_act_wrt_prev[i], temp6x6n1), temp6x6n2), F_temp, F_temp);

        if(curr_body->type == 1) {//flexible body
            if(dyn->eta != NULL){
                matrix_free(dyn->eta);
            }
            dyn->eta = zeros(6, curr_body->object->flex->N);

            if(dyn->f != NULL){
                matrix_free(dyn->f);
            }

            dyn->f = zeros(6, curr_body->object->flex->N);

            if(i == BC_Start ) {
                //ROBOT{2*i-1}.Stiff * (InitGuess - ROBOT{2*i-1}.F_0);

                setSection(F, 0, F->numRows-1, i, i, matMult(curr_body->object->flex->stiff,
                                                      matrix_sub(str_guess, curr_body->object->flex->F_0, tempR6n1),tempR6n1));

            } else{
                setSection(F,0,F->numRows-1,i,i, F_temp);
            }


            tempC6n1 = matrix_transpose(getSection(F, 0, F->numRows-1, i, i, tempR6n1), tempC6n1);
            matMult(tempC6n1, curr_joint->object->joint->twistR6, temp1);
            setSection(C, 0,0,i-1,i-1,temp1);


            if(F_dist != NULL){
                matrix_free(F_dist);
            }
            F_dist = zeros(6, curr_body->object->flex->N);


            flex_dyn(g_ref[i], F_dist, getSection(F, 0, F->numRows-1, i, i, tempR6n1), curr_body->object->flex,
                     getSection(eta, 0, eta->numRows-1, i, i, tempR6n2), c0, c1, c2, dyn);

            setSection(d_eta, 0,d_eta->numRows-1,i,i, dyn->d_eta_end);
            //memcpy(g_ref[i], dyn->g_end, sizeof(matrix));
            copyMatrix(dyn->g_end, g_ref[i]);

//            g_ref[i] = dyn->g_end;//is this the leak??
            //d_eta[i] = *dyn->d_eta_end;


            matMult(curr_body->object->flex->stiff, matrix_sub(getSection(dyn->f,0,5,dyn->f->numCols-1,dyn->f->numCols-1, tempR6n1), curr_body->object->flex->F_0, tempR6n1), F_temp);
            setSection(eta,0,eta->numRows-1,i, i, getSection(dyn->eta,0,5,dyn->eta->numCols-1, dyn->eta->numCols-1, tempR6n1));


        }else if(i>BC_Start) {//rigid bodies
            setSection(F,0,F->numRows-1,i,i, F_temp);// [N;Nm] Save Wrench Between i,i-1_th Body @ CoM Expressed in BCF




                //setSection(C);
                if(i<numBody+1) {
                    tempC6n1 = matrix_transpose(getSection(F, 0, F->numRows-1, i, i, tempR6n1), tempC6n1);
                    matMult(tempC6n1, curr_joint->object->joint->twistR6, temp1);
                    setSection(C, 0, 0, i - 1, i - 1, temp1);
                }

            /*
             * F_temp = F(:,i) + transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i)- ROBOT{2*i-1}.Mass*d_eta(:,i);
             */
            getSection(F, 0,5,i,i, tempR6n1);
            getSection(eta,0,5,i,i, tempR6n2);
            adj_R6(tempR6n2, temp6x6n1);
            matrix_transpose(temp6x6n1, temp6x6n1);
            matMult(curr_body->object->rigid->mass, getSection(eta,0,5,i,i, tempR6n3), tempR6n3);
            matMult(temp6x6n1, tempR6n3, tempR6n2);
            matrix_add(tempR6n1, tempR6n2, tempR6n1);


            matMult(curr_body->object->rigid->mass, getSection(d_eta,0,5,i,i, tempR6n2), tempR6n3);
            matrix_sub(tempR6n1
                    ,tempR6n3, F_temp);

        }

    }




//    F(:,end) = F_ext;
//    for i = flip(2 : BC_Start)
//    F(:,i-1) = transpose(Ad(g_act_wrt_prev(:,:,i+1))) * F(:,i) + ...
//    ROBOT{2*i-1}.Mass*d_eta(:,i) - transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i);
//    C(:,i-1) = transpose(F(:,i-1)) * Ad(expm3(hat(-ROBOT{2*i-1}.CoM)))*ROBOT{2*(i-1)}.Twist;      %[] Transform Joint twist to Link CoM BCF then Project Constraints
//    end

    setSection(F, 0,5,F->numCols - 1,F->numCols - 1, F_ext);
    for(int i = BC_Start; i >= 1; i--) {
        curr_joint = robot->objects[2 * (i - 1)+1];
        curr_body = robot->objects[2 * i ];

        //transpose(Ad(g_act_wrt_prev(:,:,i+1))) * F(:,i) +

        adj(g_act_wrt_prev[i+1], temp6x6n1);
        matrix_transpose(temp6x6n1, temp6x6n1);

        getSection(F, 0,5,i,i, tempR6n2);

        matMult(temp6x6n1, tempR6n2, tempR6n1);

        if(curr_body->type == 1) {//flexible body
            //temp6x6n2 = curr_body->object->flex->mass;
            copyMatrix(curr_body->object->flex->mass, temp6x6n2);
        }else if(curr_body->type == 0){
            copyMatrix(curr_body->object->rigid->mass, temp6x6n2);
            //temp6x6n2 = curr_body->object->rigid->mass;
        }else{
            assert(1 == 0);//this is a bad way to do errors
        }



        getSection(d_eta, 0,5,i,i, tempR6n3);
        matMult(temp6x6n2, tempR6n3, tempR6n3);
        //-
        //tempr6n3
        getSection(eta, 0,5,i,i, tempR6n4);
        adj_R6(tempR6n4, temp6x6n1);
        matrix_transpose(temp6x6n1, temp6x6n1);
        matMult(temp6x6n2, temp6x6n1, temp6x6n1);
        //mass
        getSection(eta, 0,5,i,i, tempR6n4);
        matMult(temp6x6n1, tempR6n4, tempR6n2);


        matrix_sub(tempR6n3, tempR6n2, tempR6n2);//second line
        matrix_add(tempR6n1, tempR6n2, tempR6n1);//full line
        setSection(F, 0,5,i-1,i-1, tempR6n1);



        if(curr_body->type == 1) {//flexible body
            //temp6x6n2 = curr_body->object->flex->mass;
            copyMatrix(curr_body->object->flex->CoM, tempR6n3);
        }else if(curr_body->type == 0){
            copyMatrix(curr_body->object->rigid->CoM, tempR6n3);
            //temp6x6n2 = curr_body->object->rigid->mass;
        }else{
            assert(1 == 0);//this is a bad way to do errors
        }

        //C(:,i-1) = transpose(F(:,i-1)) * Ad(expm3(hat(-ROBOT{2*i-1}.CoM)))*ROBOT{2*(i-1)}.Twist;

        getSection(F, 0,5,i-1,i-1, tempR6n1);
        matrix_transpose(tempR6n1, tempR6t);

        matrix_scalar_mul( tempR6n3, -1, tempR6n3);
        hat_R6(tempR6n3, temp4x4n1);
        expm_SE3(temp4x4n1, temp4x4n1);
        adj(temp4x4n1, temp6x6n1);

        matMult(tempR6t, temp6x6n1, tempR6t);

        copyMatrix(curr_joint->object->joint->twistR6, tempR6n2);

        matMult(tempR6t, tempR6n2, temp1);
        setSection(C, 0,0,i-1,i-1,temp1);



    }
    matrix *C_inv = matrix_new(C->numCols,1);
    matrix_transpose(C, C_inv);

    matrix_sub(C_des, C_inv, C_inv);



    num = 0;
    for(int i = 0; i < (robot->numObjects/2)-1; i++){
        if(robot->objects[(i*2)+1]->type == 2){

            robot->objects[(2*i)+1]->object->joint->acceleration = accel_old->data[num];


            robot->objects[(2*i)+1]->object->joint->velocity = vel_old->data[num];
            //printf("%f", robot->objects[(2*i)+1]->object->joint->velocity);
            num++;
        }
    }
    //matrix *tempT = matrix_new(1, 5);
    //matrixToFile(C, "C.csv");

    assert(isnan(robot->objects[1]->object->joint->velocity)==0);
    copyMatrix(C_inv, result);

    #if LOG_F_FLEX == 1
    matrix* temp = matrix_new(1,5);
    matrix_transpose(C_inv, temp);
    matrixToFile(temp, "C_inv.csv");
    matrix_transpose(InitGuess, temp);
    matrixToFile(temp, "InitGuess.csv");
    #endif
    return GSL_SUCCESS;




}


// Define the function whose roots we want to find
int Flex_MB_BCS_wrapper(const gsl_vector *x, void *params, gsl_vector *f) {
    Flex_MB_BCS_params *p = (Flex_MB_BCS_params *)params;


    // Extracting the elements of x
    double x_arr[6];
    for (int i = 0; i < 6; ++i) {
        x_arr[i] = gsl_vector_get(x, i);
    }

    // Convert x to matrix format
    matrix *x_matrix = zeros(6, 1);
    x_matrix->data[(0 * x_matrix->numCols)] = x_arr[0];
    x_matrix->data[(1 * x_matrix->numCols)] = x_arr[1];
    x_matrix->data[(2 * x_matrix->numCols)] = x_arr[2];
    x_matrix->data[(3 * x_matrix->numCols)] = x_arr[3];
    x_matrix->data[(4 * x_matrix->numCols)] = x_arr[4];
    x_matrix->data[(5 * x_matrix->numCols)] = x_arr[5];

    // Call Flex_MB_BCS function
    matrix *result;

    //assert(hasNan(x_matrix) == 0);
    result = Flex_MB_BCS(x_matrix, params);
    // Fill f with the residuals
    for (int i = 0; i < 6; ++i) {
        //printf("result: %f\n", result->data[i][0]);
        gsl_vector_set(f, i, result->data[(i * result->numCols)] );
    }
    if(hasNan(x_matrix) == 1){
        return GSL_EINVAL;
    }
    // Free memory
    matrix_free(x_matrix);
    matrix_free(result);

    return GSL_SUCCESS;//todo is this right?
}

void Flex_MB_BCS_wrapper_levmar(double *x, double *f, int m, int n, void *params) {
    Flex_MB_BCS_params *p = (Flex_MB_BCS_params *)params;

    matrix *initGuess = matrix_new(6, 1);
    for (int i = 0; i < 6; ++i) {
        initGuess->data[i * initGuess->numCols] = x[i];
    }


    matrix *result;
    result = Flex_MB_BCS(initGuess, params);

    for (int i = 0; i < 6; ++i) {
        f[i] = result->data[i * result->numCols];
    }

}



// Define the function whose roots we want to find
int F_Flex_MB_BCS_wrapper(const gsl_vector *x, void *params, gsl_vector *f) {




    // Extracting the elements of x
    int code = -1;
    double x_arr[x->size];
    for (int i = 0; i < x->size; ++i) {
        x_arr[i] = gsl_vector_get(x, i);
    }

    // Convert x to matrix format
    matrix *x_matrix = zeros(x->size, 1);
    for (int i = 0; i < x_matrix->numRows; ++i) {
        x_matrix->data[i * x_matrix->numCols] = x_arr[i];
    }

    // Call Flex_MB_BCS function
    matrix *result = zeros(f->size, 1);
    //assert(hasNan(x_matrix) == 0);
    code = F_Flex_MB_BCS(x_matrix, result, params);
    // Fill f with the residuals
    for (int i = 0; i < f->size; ++i) {
        //printf("result: %f\n", result->data[i][0]);
        gsl_vector_set(f, i, result->data[(i * result->numCols)] );
    }
    if(hasNan(x_matrix) == 1){
        return GSL_EINVAL;
    }
    // Free memory
    matrix_free(x_matrix);
    matrix_free(result);
    return code;

    return GSL_SUCCESS;//todo is this right?
}

void F_Flex_MB_BCS_wrapper_levmar(double *x, double *f, int m, int n, void *params) {
    Flex_MB_BCS_params *p = (Flex_MB_BCS_params *)params;

    matrix *initGuess = matrix_new(n, 1);
    for (int i = 0; i < n; ++i) {
        initGuess->data[i * initGuess->numCols] = x[i];
    }


    matrix *result = matrix_new(m, 1);
    F_Flex_MB_BCS(initGuess, result, params);

    for (int i = 0; i < n; ++i) {
        f[i] = result->data[i * result->numCols];
    }

}

int find_roots_levmarqrt(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, double tol) {
//
//    Flex_MB_BCS_params params = {robot, Theta, Theta_dot, Theta_DDot, F_ext, c0, c1, c2};
    //dlevmar_dif(meyer, p, x, m, n, 1000, opts, info, work, covar, NULL); // no



    int n = InitGuess->numRows;
    double *p = malloc(n * sizeof(double));
    for (int i = 0; i < n; ++i) {
        p[i] = InitGuess->data[i * InitGuess->numCols];
    }
    double x[n];
    for(int i = 0; i < n; i++){
        x[i] = InitGuess->data[i * InitGuess->numCols];
    }
    double *info = (double *)malloc(10 * sizeof(double));
    double opts[5];
    /* I: opts[0-4] = minim. options [\mu, \epsilon1, \epsilon2, \epsilon3, \delta]. Respectively the
                       * scale factor for initial \mu,
                       * stopping thresholds for ||J^T e||_inf,
                       * ||Dp||_2 and
                       * ||e||_2 and
                       * the step used in difference approximation to the Jacobian. Set to NULL for defaults to be used.
                       * If \delta<0, the Jacobian is approximated with central differences which are more accurate
                       * (but slower!) compared to the forward differences employed by default.
                       */
    /*
     *
     * 0: scale factor for initial mu
     */
    if(fwd) {
        //opts[5] = {1e-3, 1e-9, 1e-5, 1e-5, -1e-9};
        opts[0] = 1e-6; // scale factor for initial mu
        opts[1] = 1e-15; // stopping thresholds for ||J^T e||_inf
        opts[2] = D_P_LEVMAR; // stopping thresholds for ||Dp||_2
        opts[3] = tol; // stopping thresholds for ||e||_2
        opts[4] = STEP_LEVMAR; // the step used in difference approximation to the Jacobian

    }else{
        //{1e-3, 1e-15, 1e-9, 1e-9, 1e-6};

        opts[0] = 1e-3;
        opts[1] = 1e-15;
        opts[2] = D_P_LEVMAR;
        opts[3] = 1e-12;
        opts[4] = STEP_LEVMAR;
    }
    assert(isnan(params->robot->objects[1]->object->joint->velocity)==0);
    if(fwd){
        dlevmar_dif(F_Flex_MB_BCS_wrapper_levmar, p, NULL, 5, 5, MAX_ITER_LEVMAR, opts, info, NULL, NULL, params);
    }else {
        dlevmar_dif(Flex_MB_BCS_wrapper_levmar, p, NULL, 6, 6, MAX_ITER_LEVMAR, opts, info, NULL, NULL, params);//TODO SOMETING ON THIS LINE MADE Inv Dyn work. not sure what, changed them all to main
    }

    #if SOLVER_SAVE
    if(fwd){
        solverSave("levmar", "levmarSaveFwd.csv", (int)info[6], (int)info[5], (double) info[1], fwd);
    }else {
        solverSave("levmar", "levmarSave.csv", (int)info[6], (int)info[5], (double) info[1], fwd);
    }
    #endif

    #if VERBOSE >= 2
    printf("iters: %f\n", info[5]);
    printf("reason for terminating: %f\n", info[6]);

    switch ((int)info[6]) {
        case 1:
            printf("stopped by small gradient J^T e\n");
            break;
        case 2:
            printf("stopped by small Dp.\n");
            break;
        case 3:
            printf("max number of iterations reached\n");
            break;
        case 4:
            printf("singular matrix. Restart from current p with increased mu\n");
            break;
        case 5:
            printf("no further error reduction is possible. Restart with increased mu\n");
            break;
        case 6:
            printf("stopped by small ||e||_2 i.e. solver converged\n");
            break;
        case 7:
            printf("stopped by invalid (i.e. NaN or Inf) function values. Restart with current p\n");
            break;
        case 8:
            printf("stopped by parameter limits\n");
            break;
        default:
            printf("unknown reason for termination\n");
            break;

    }


    printf("\te_2 @initial p %.30f\n", info[0]);
    printf("\te_2 %.30f\n", info[1]);
    printf("\tJ^T e %.30f\n", info[2]);
    printf("\t||Dp||_2 %.30f\n", info[3]);
    printf("\tmu/max[J^T J] %.30f\n", info[4]);
    printf("\titers: %f\n", info[5]);
    printf("\tevals: %f\n", info[7]);
    printf("\tjacobian evals: %f\n", info[8]);
    printf("\tlinear system solves: %f\n", info[9]);

    printf("\tSolution\n");
    printMatrix(InitGuess);
    printf("the residual is: %.30f\n", info[1]);
    #endif
    for (int i = 0; i < InitGuess->numRows; ++i) {
        InitGuess->data[i * InitGuess->numCols] = p[i];
    }
    return info[6];
}
//(lldb) br set --name malloc_error_break
//        (lldb) br set -n malloc_error_break

int find_roots_newton(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, double tol) {
    const gsl_multiroot_fsolver_type *T;
    gsl_multiroot_fsolver *s;

    T = gsl_multiroot_fsolver_dnewton;
    //s = gsl_multiroot_fsolver_allc(T, 6);
    int status;
    size_t iter = 0;

    const size_t n = InitGuess->numRows; // Number of variables




    gsl_multiroot_function *f = malloc(sizeof(gsl_multiroot_function));
    // Set parameters
    if(fwd == 1){
        f->f = &F_Flex_MB_BCS_wrapper;
        f->n = n;
        f->params = params;

    }else {
        f->f = &Flex_MB_BCS_wrapper;
        f->n = n;
        f->params = params;
    }
    //f.params = &params;

    // Define initial guess
    double x_init[n];

    for (int i = 0; i < n; ++i) {
        x_init[i] = InitGuess->data[i * InitGuess->numCols];
    }

    gsl_vector_view x_vec = gsl_vector_view_array(x_init, n);
    s = gsl_multiroot_fsolver_alloc(T, n);
    gsl_multiroot_fsolver_set(s, f, &x_vec.vector);


    #if VERBOSE >= 2
    // output the jacobian
    gsl_matrix *jacobian = gsl_matrix_alloc(n, n);
    int stat = gsl_multiroot_fdjacobian(f, &x_vec.vector, s->f, 1e-9, jacobian);
//    printf("JACOBIAN:\n");
//    printf("STATUS: %d\n", stat);
//    printGSLMatrix(jacobian);
//    printf("determinant of the jacobian is: %.15f", Det(jacobian));
    #endif
    do {
        iter++;
        status = gsl_multiroot_fsolver_iterate(s);
        #if VERBOSE >= 2
        if (status) {
            printf("STATUS: %s\n", gsl_strerror(status));
            break;
        }
        #endif


        status = gsl_multiroot_test_residual(s->f, tol);


    } while (status == GSL_CONTINUE && iter < MAX_ITER_NEWTON);


#if SOLVER_SAVE ==1
    double residual = 0;

    residual = gsl_blas_dnrm2(s->f);

    if(fwd){
        solverSave("newton", "newtonSaveFwd.csv", status, (int)iter, residual, 1);
    }else {
        solverSave("newton", "newtonSave.csv", status, (int) iter, residual, fwd);
    }
#endif

    #if VERBOSE >= 2
    if (status) {
        printf("STATUS: %d\n", status);
        printf("STATUS: %s\n", gsl_strerror(status));
    }else{
        printf("Newton Solver Converged in %zu iterations\n", iter);
    }
    #endif

    //printf("took %zu iterations\n", iter);
    //assert(!isnan(s->f->data[0]));
    // Extract solution
    //matrix *solution = zeros(6, 1);
    for (int i = 0; i < InitGuess->numRows; ++i) {
        InitGuess->data[i * InitGuess->numCols] = gsl_vector_get(s->x, i);
    }

    gsl_multiroot_fsolver_free(s);
    free(f);

    return status;
}

int find_roots_hybrid(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, double tol) {
    const gsl_multiroot_fsolver_type *T;
    //gsl_multiroot_fsolver *s;
    assert(hasNan(InitGuess) == 0);
    T = gsl_multiroot_fsolver_hybrids;
    //s = gsl_multiroot_fsolver_allc(T, 6);
    int status;
    size_t iter = 0;

    const size_t n = InitGuess->numRows; // Number of variables



    gsl_multiroot_function f = {&Flex_MB_BCS_wrapper, n, params};
    // Set parameters
    if(fwd) {

        f.f = &F_Flex_MB_BCS_wrapper;
    }

    double x_init[InitGuess->numRows];

    for (int i = 0; i < InitGuess->numRows; ++i) {
        x_init[i] = InitGuess->data[i * InitGuess->numCols];
    }

    gsl_vector_view x_vec = gsl_vector_view_array(x_init, n);
    gsl_multiroot_fsolver *s = gsl_multiroot_fsolver_alloc(T, n);
    gsl_multiroot_fsolver_set(s, &f, &x_vec.vector);


    do {
        iter++;
        status = gsl_multiroot_fsolver_iterate(s);

        #if VERBOSE >= 2
        if (status) {
            printf("STATUS: %s\n", gsl_strerror(status));
            break;
        }
        #endif


        status = gsl_multiroot_test_residual(s->f, tol);

//        if(status == GSL_CONTINUE){
//            if(fwd) {
//                status = gsl_multiroot_test_delta(s->dx, s->x, TOLERANCE_STEP, TOLERANCE_STEP);
//            }else {
//                status = gsl_multiroot_test_delta(s->dx, s->x, TOLERANCE_STEP, TOLERANCE_STEP);
//            }
//        }

    } while (status == GSL_CONTINUE && iter < MAX_ITER_HYBRID);


#if SOLVER_SAVE ==1
    double residual = 0;
    residual = gsl_blas_dnrm2(s->f);
    if(fwd){
        solverSave("hybrid", "hybridSaveFwd.csv", status, (int)iter, residual, 1);
    }else {
        solverSave("hybrid", "hybridSave.csv", status, (int) iter, residual, fwd);
    }
#endif
    #if VERBOSE >= 2
    if(fwd) {
        if (status != GSL_SUCCESS) {
            printf("\tSTATUS: %d\n", status);
            printf("\tSTATUS: %s\n", gsl_strerror(status));
        }
        printf("\thybrid took %zu iterations\n", iter);\

    }
    else{
            if (status != GSL_SUCCESS){
                printf("STATUS: %d\n", status);
                printf("STATUS: %s\n", gsl_strerror(status));
            }
            printf("hybrid took %zu iterations\n", iter);
    }
//    gsl_matrix *jacobian = gsl_matrix_alloc(n, n);
//    int stat = gsl_multiroot_fdjacobian(&f, &x_vec.vector, s->f, 1e-4, jacobian);
//    printf("JACOBIAN:\n");
//    printf("STATUS: %d\n", stat);
//    printGSLMatrix(jacobian);
//
//    matrix *determinant = zeros(6,6);
//    gsl_to_matrix(jacobian, determinant);
//    printf("DETERMINANT: %.15f\n", Det(determinant));
//    assert(fabs(Det(determinant)) > 0.0000000000000000000000000001);
    #endif



   //assert(!isnan(s->f->data[1]));
    // Extract solution
    //matrix *solution = zeros(6, 1);
    for (int i = 0; i < InitGuess->numRows; ++i) {
        InitGuess->data[i * InitGuess->numCols] = gsl_vector_get(s->x, i);
    }

    gsl_multiroot_fsolver_free(s);
    return status;
}

//int find_roots_deriv(matrix InitGuess, F)


/*
 * Inverse Dynamic Model for Rigid-Flexible Multi-Body Open-Chain Manipulator
 * robot:       Robot Definition
 * Theta:       Joint Position                          <1xnumJoints>
 * Theta_dot:   Joint Velocity                          <1xnumJoints>
 * Theta_DDot:  Joint Acceleration                      <1xnumJoints>
 * F_ext:       External Wrench                         todo <6x1> right?
 * dt:          Time Step                               double
 * InitGuess:   Initial Guess for the Solution          todo <6x1> right?
 * Returns:     Constraint, Actutation Forces & Body Velocities
 *
 * InitGuess:      Initial Guess for Strain at the Beginning of 1st Flexible Body
 * ROBOT:          Definition of Manipulator (Collection of Bodies and Joints)
 * THETA:          Actuated Joint Positions
 * THETA_DOT:      Actuated Joint Velocities
 * THETA_DDOT:     Actuated Joint Accelerations
 * F_ext:          Applied Load at the EE expressed in EE BCF
 * c0:             FDM Coeff - Current Time Step
 * c1:             FDM Coeff - Previous Time Step
 * c2:             FDM Coeff - PrePrevious Time Step
 * RRC:            Robot Reference Configuration
 * RAC:            Robot Actuated Configuration
 * BCF:            Body Coordinate Frame
 */
IDM_MB_RE_OUT *IDM_MB_RE(Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double dt, matrix *InitGuess) {

    int numBody = 5;//todo this should not be a magic number
    int BC_Start = getBCStart(robot);
    //int BC_End = getBCEnd(robot);

    if (BC_Start == -1) {
        //todo not sure what to do here, it might just work?
        printf("No Flexible Body Found\n");
    }


    //todo implement 3d zeros()
    //todo this might not need +2, I dont remember if numBodies includes base and EE
    matrix **g_ref = malloc(sizeof(matrix) * (numBody +
                                              2));           //[SE(3) X N+2]  Transformation to i_th C-BCF from/in base BCF for RRC
    for (int i = 0; i < numBody + 2; i++) {
        g_ref[i] = zeros(4, 4);
    }
    matrix **g_act_wrt_prev = malloc(
            sizeof(matrix) * (numBody + 2));  //[SE(3) X N+2]  Transformation to i-1_th C-BCF from/in i_th BCF for RAC
    for (int i = 0; i < numBody + 2; i++) {
        g_act_wrt_prev[i] = zeros(4, 4);
    }

    matrix *eta = zeros(6, numBody + 2);               //[se(3) X N+2]  Twists for each BCF + Base + EE in BCF
    matrix *d_eta = zeros(6, numBody + 2);             //[se(3) X N+2]  Twist Rate for each BCF + Base + EE Frame in BCF
    matrix *F = zeros(6, numBody + 2);                 //[se(3) X N+1]  Wrench for each Joint + EE in BCF
    matrix *C = zeros(1, numBody);                     //[se(3) X N]    Actuated Control for each Joint in BCF

    matrix *CoM2CoM = zeros(4, 4);
    //[]     FDM Coefficients for BDF-2
    double c0 = 1.5 / dt;//60
    double c1 = -2 / dt;//-80
    double c2 = .5 / dt;//20
    //options = optimset('Display','OFF','TolFun',1e-9);

    //todo does this need to find ALL roots or just the one 'nearest' to the initial guess?
    //matrix *InitGuess = fsolve(@(InitGuess)Flex_MB_BCS(InitGuess, ROBOT, THETA, THETA_DOT, ...
    //THETA_DDOT, F_ext, c0, c1, c2),InitGuess,options);
    //printf("INIT_GUESS pre\n");

    //printMatrix(matrix_sub(ones(6,1),InitGuess));
    //printMatrix(Flex_MB_BCS(InitGuess, robot, *F_ext, c0,c1,c2 ));
    //assert(!isnan(InitGuess->data[0][0]));

    Flex_MB_BCS_params *params = malloc(sizeof(Flex_MB_BCS_params));
    params->robot = robot;
    params->F_ext = F_ext;
    params->c0 = c0;
    params->c1 = c1;
    params->c2 = c2;
    params->dt = dt;
    params->Theta = Theta;
    params->Theta_dot = Theta_dot;
    params->Theta_ddot = Theta_DDot;
    params->C_des = NULL;
    params->inv = 0;
    params->F_0 = matrix_new(6, 1);

    matrix *tempGuess = matrix_new(6, 1);
    copyMatrix(InitGuess, tempGuess);
    matrix *tempT = matrix_new(1, 6);
    matrixToFile(matrix_transpose(InitGuess, tempT), "idmSolveInit.csv");
#if VERBOSE >= 1

    printf("____________InitGuess_______________________\n");
    printMatrix(InitGuess);

#endif
    assert(hasNan(tempGuess)==0);
    int status = find_roots_hybrid(tempGuess, params, 0, TOLERANCE_INV);

    if (status != 0) {
#if VERBOSE >= 1
        printf("hybrid method failed to converge. Trying levmar\n");
        //copyMatrix(InitGuess, tempGuess);
#endif
        status = find_roots_levmarqrt(tempGuess, params, 0, TOLERANCE_INV);
        //copyMatrix(tempGuess, InitGuess);
        if (status != 6) {
            printf("levmar method failed to converge trying newton\n");
            //copyMatrix(InitGuess, tempGuess);
            status = find_roots_newton(tempGuess, params, 0, TOLERANCE_INV);
            //copyMatrix(tempGuess, InitGuess);
            if(status != 0){
                printf("newton method failed to converge. ALL FAILED");
                #if SOLVER_ERRORS == 1
                assert(1 == 0);
                #endif

            }
        }
    }
        //printMatrix(Theta);
    copyMatrix(tempGuess, InitGuess);


        #if VERBOSE >= 1

        printf("_______________SOLUTION______________________\n");
        printMatrix(InitGuess);
        printf("___________________________________________\n\n");
        #endif
        eye(g_ref[0]);
        eye(g_act_wrt_prev[0]);

    setSection(eta, 0, 5, 0, 0, zeros(6, 1));
    setSection(d_eta, 0, 5, 0, 0, zeros(6, 1));

        matrix *F_temp = zeros(6, 1);
        Theta = zeros(numBody, 1);
        setSection(Theta, 0, numBody - 1, 0, 0, Theta);

        rigidKin *kin = rigidKinAlloc();
        matrix *F_dist;
        flexDyn *dyn = flexDynAlloc();


        matrix **etaPrev = malloc(sizeof(matrix) * (numBody + 2));
        for (int i = 0; i < numBody + 2; i++) {//todo this could be numFlex I think
            etaPrev[i] = zeros(6, 21);
        }

        matrix **etaPPrev = malloc(sizeof(matrix) * (numBody + 2));
        for (int i = 0; i < numBody + 2; i++) {//todo this could be numFlex I think
            etaPPrev[i] = zeros(6, 21);
        }

        matrix **fPrev = malloc(sizeof(matrix) * (numBody + 2));
        for (int i = 0; i < numBody + 2; i++) {//todo this could be numFlex I think
            fPrev[i] = zeros(6, 21);
        }
        matrix **fPPrev = malloc(sizeof(matrix) * (numBody + 2));
        for (int i = 0; i < numBody + 2; i++) {//todo this could be numFlex I think
            fPPrev[i] = zeros(6, 21);
        }


        matrix *tempR6n1 = matrix_new(6, 1);
        matrix *tempR6n2 = matrix_new(6, 1);
        matrix *tempR6n3 = matrix_new(6, 1);
        matrix *tempR6n4 = matrix_new(6, 1);

        matrix *tempR6n1t = matrix_new(1, 6);

        matrix *temp4x4n1 = matrix_new(4, 4);


        matrix *tempC6n1 = matrix_new(1, 6);
        matrix *temp6x6n1 = matrix_new(6, 6);
        matrix *temp6x6n2 = matrix_new(6, 6);

        for (int i = 1; i < robot->numBody + 2; i++) {
            //printMatrix(C);
            //printf("\n\n");
            rigidJoint *joint = robot->objects[2 * (i - 1) + 1]->object->joint;
            Object *body = robot->objects[2 * i];
            assert(robot->objects[2 * (i - 1) + 1]->type == 2);
            assert(robot->objects[2 * i - 2]->type == 1 || robot->objects[2 * i - 2]->type == 0);


            CoM2CoM = getCoM2CoM(joint, CoM2CoM);
//        printf("CoM2CoM\n");
//        printMatrix(CoM2CoM);
//        printf("g_ref[i - 1]");
//        printMatrix(g_ref[i - 1]);
            zeroMatrix(tempR6n1);

            getSection(eta, 0, 5, i - 1, i - 1, tempR6n1);

            actuateRigidJoint(g_ref[i - 1], CoM2CoM, joint,
                              tempR6n1, getSection(d_eta, 0, 5, i - 1, i - 1, tempR6n2), kin);

            copyMatrix(kin->g_act_wrt_prev, g_act_wrt_prev[i]);
            //g_act_wrt_prev[i] = kin->g_act_wrt_prev;
            copyMatrix(kin->g_cur, g_ref[i]);
            //g_ref[i] = kin->g_cur;

            setSection(eta, 0, 5, i, i, kin->eta);
            setSection(d_eta, 0, 5, i, i, kin->d_eta);

//        printf("gref-1\n");
//        printMatrix(g_ref[i-1]);
//
//        printf("gref\n");
//        printMatrix(g_ref[i]);
            matMult(matrix_transpose(adj(g_act_wrt_prev[i], temp6x6n1), temp6x6n2), F_temp, F_temp);

            if (body->type == 1) {//flexible body
                if (dyn->eta != NULL) {
                    matrix_free(dyn->eta);
                }
                dyn->eta = zeros(6, body->object->flex->N);

                if (dyn->f != NULL) {
                    matrix_free(dyn->f);
                }
                dyn->f = zeros(6, body->object->flex->N);
                if (i == BC_Start) {//todo double check this +1

                    setSection(F, 0, 5, i, i, matMult(body->object->flex->stiff,
                                                      matrix_sub(InitGuess, body->object->flex->F_0, tempR6n1),
                                                      tempR6n1));
                } else {
                    setSection(F, 0, 5, i, i, F_temp);
                }

                tempC6n1 = matrix_transpose(getSection(F, 0, 5, i, i, tempR6n1), tempC6n1);
                //setSection(C);
                matrix *temp1 = matrix_new(1,1);
                matMult(tempC6n1, joint->twistR6, temp1);
                setSection(C, 0,0,i-1,i-1,temp1);

                F_dist = zeros(6, body->object->flex->N);
                flex_dyn(g_ref[i], F_dist, getSection(F, 0, 5, i, i, tempR6n1), body->object->flex,
                         getSection(eta, 0, 5, i, i, tempR6n2), c0, c1, c2, dyn);

                setSection(d_eta, 0, 5, i, i, dyn->d_eta_end);//added this

                //g_ref[i] = dyn->g_end;
                copyMatrix(dyn->g_end, g_ref[i]);
                matMult(body->object->flex->stiff,
                        matrix_sub(getSection(dyn->f, 0, 5, dyn->f->numCols - 1, dyn->f->numCols - 1, tempR6n1),
                                   body->object->flex->F_0, tempR6n1), F_temp);


                setSection(eta, 0, 5, i, i,
                           getSection(dyn->eta, 0, 5, dyn->eta->numCols - 1, dyn->eta->numCols - 1, tempR6n1));


                //update history terms AFTER calculations
                copyMatrix(body->object->flex->f_prev, fPPrev[i]);
                //fPPrev[i] = body->object->flex->f_prev;
                copyMatrix(dyn->f, fPrev[i]);
                //fPrev[i] = dyn->f;

                copyMatrix(body->object->flex->eta_prev, etaPPrev[i]);//double che
                //etaPPrev[i] = *body->object->flex->eta_prev;//double check this

                copyMatrix(dyn->eta, etaPrev[i]);
                //etaPrev[i] = *dyn->eta;


            } else if (i > BC_Start) {//rigid bodies



                setSection(F, 0, 5, i, i, F_temp);// [N;Nm] Save Wrench Between i,i-1_th Body @ CoM Expressed in BCF

                if (i < numBody + 2) {

                    //setSection(C, 0,5, i - 1, i - 1, matMult(matrix_transpose(getSection(F, 0,5,i,i)), robot->objects[2*i-2]->object->joint->twistR6));

                    tempC6n1 = matrix_transpose(getSection(F, 0, 5, i, i, tempR6n1), tempC6n1);
                    matrix *temp1 = matrix_new(1,1);
                    matMult(tempC6n1, joint->twistR6, temp1);
                    setSection(C, 0,0,i-1,i-1,temp1);

                }

                if (body->type == 1) {//flex
                    F_temp = matrix_sub(matrix_add(getSection(F, 0, 5, i, i, tempR6n1),
                                                   matMult(matMult(matrix_transpose(
                                                                           adj_R6(getSection(eta, 0, 5, i, i, tempR6n2), temp6x6n1),
                                                                           temp6x6n1),
                                                                   body->object->flex->mass, temp6x6n1),
                                                           getSection(eta, 0, 5, i, i, tempR6n3), tempR6n2), tempR6n1),
                                        matMult(body->object->flex->mass, getSection(eta, 0, 5, i, i, tempR6n2),
                                                tempR6n2), F_temp);//todo double check there is no overlap

//                F_temp = matMult(matrix_add(getSection(F, 0, 5, i, i), matrix_transpose(adj_R6(getSection(eta, 0, 5, i, i)))),
//                        matMult(curr_body->object->flex->mass, getSection(eta, 0, 5, i, i)));
            } else {
                F_temp = matrix_sub(matrix_add(getSection(F, 0, 5, i, i, tempR6n1),
                                               matMult(matMult(matrix_transpose(adj_R6(getSection(eta, 0, 5, i, i, tempR6n2), temp6x6n1), temp6x6n1),
                                                               body->object->rigid->mass, temp6x6n1), getSection(eta, 0, 5, i, i, tempR6n2), tempR6n2), tempR6n2),
                                    matMult(body->object->rigid->mass, getSection(d_eta, 0, 5, i, i, tempR6n3), tempR6n3), F_temp);


                }

            }

        }
        setSection(F, 0, 5, F->numCols - 1, F->numCols - 1, F_ext);
        if (BC_Start < numBody) {
            matrix *objMass = malloc(sizeof(matrix));
            matrix *objCoM = malloc(sizeof(matrix));


            for (int i = BC_Start; i >= 1; i--) {
                rigidJoint *joint = robot->objects[2 * (i - 1) + 1]->object->joint;
                Object *body = robot->objects[2 * i];
                if (body->type == 1) {
                    objMass = body->object->flex->mass;
                    objCoM = body->object->flex->CoM;
                } else if (body->type == 0) {
                    objMass = body->object->rigid->mass;
                    objCoM = body->object->rigid->CoM;
                }
                setSection(F, 0, 5, i - 1, i - 1,
                           matrix_add(
                                   matMult(matrix_transpose(adj(g_act_wrt_prev[i + 1], temp6x6n1), temp6x6n1),
                                           getSection(F, 0, 5, i, i, tempR6n1), tempR6n1),
                                   matrix_sub(matMult(objMass, getSection(d_eta, 0, 5, i, i, tempR6n2), tempR6n2),
                                              matMult(matrix_transpose(
                                                              adj_R6(getSection(eta, 0, 5, i, i, tempR6n3), temp6x6n2),
                                                              temp6x6n2),
                                                      matMult(objMass, getSection(eta, 0, 5, i, i, tempR6n4), tempR6n4),
                                                      tempR6n3), tempR6n2),
                                   tempR6n1));


            setSection(C, 0, C->numRows - 1, i-1, i-1,
                       matMult(
                       matrix_transpose(getSection(F, 0, 5, i-1, i-1, tempR6n1), tempR6n1t),
                       matMult(
                               adj(expm_SE3(hat_R6(matrix_scalar_mul(objCoM,-1, tempR6n1),temp4x4n1), temp4x4n1), temp6x6n1),
                               joint->twistR6,
                               tempR6n2),
                               temp6x6n2
                       ));

            }
            //matrix_free(objMass);
            //matrix_free(objCoM);
        }

        matrix *Ct = matrix_new(C->numCols, C->numRows);
        matrix_transpose(C, Ct);
        //matrix_free(C);
        for (int i = 1; i < numBody + 2; i++) {
            Object *body = robot->objects[2 * i];
            if (body->type == 1) {
                copyMatrix(etaPrev[i], body->object->flex->eta_prev);
                copyMatrix(etaPPrev[i], body->object->flex->eta_pprev);

                copyMatrix(fPrev[i], body->object->flex->f_prev);
                copyMatrix(fPPrev[i], body->object->flex->f_pprev);
                //body->object->flex->f_prev = fPrev[i];
                //body->object->flex->f_pprev = fPPrev[i];

            }

        }

        //free(dyn);//todo write free_flexDyn to fully free
        //free(kin);//todo write free_rigidKin to fully free

        IDM_MB_RE_OUT *out = (IDM_MB_RE_OUT *) malloc(sizeof(IDM_MB_RE_OUT));
        out->C = Ct;
        out->F = F;
        out->v = eta;
        //out->robot_new = robot;


        for (int i = 0; i < numBody + 2; i++) {
            matrix_free(g_ref[i]);
            matrix_free(g_act_wrt_prev[i]);
        }

        freeRigidKin(kin);
        //matrix_free(Ct);
        matrix_free(tempR6n1);
        matrix_free(tempR6n2);
        matrix_free(tempR6n3);
        matrix_free(tempR6n4);

        matrix_free(temp6x6n1);
        matrix_free(temp6x6n2);
        matrix_free(tempR6n1t);

        matrix_free(temp4x4n1);
        matrix_free(F_dist);

        matrix_free(d_eta);
        return out;

    }



void custom_error_handler(const char *reason, const char *file, int line, int gsl_errno) {
    fprintf(stderr, "GSL Error: %s:%d: %s (code %d)\n", file, line, reason, gsl_errno);

}

FDM_MB_RE_OUT *FDM_MB_RE(Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double dt, matrix *C_des, matrix *F_0, matrix *Theta_DDot_guess) {

    #if GSL_ERROR_HANDLER == 1
        gsl_set_error_handler(&custom_error_handler);
    #endif
    int numBody = 5;//todo this should not be a magic number
    int BC_Start = getBCStart(robot);
    //int BC_End = getBCEnd(robot);

    if (BC_Start == -1) {
        //todo not sure what to do here, it might just work?
        printf("No Flexible Body Found\n");
    }


    //todo implement 3d zeros()
    //todo this might not need +2, I dont remember if numBodies includes base and EE
    matrix **g_ref = malloc(sizeof(matrix) * (numBody +
                                              2));           //[SE(3) X N+2]  Transformation to i_th C-BCF from/in base BCF for RRC
    for (int i = 0; i < numBody + 2; i++) {
        g_ref[i] = zeros(4, 4);
    }
    matrix **g_act_wrt_prev = malloc(
            sizeof(matrix) * (numBody + 2));  //[SE(3) X N+2]  Transformation to i-1_th C-BCF from/in i_th BCF for RAC
    for (int i = 0; i < numBody + 2; i++) {
        g_act_wrt_prev[i] = zeros(4, 4);
    }

    matrix *eta = zeros(6, numBody + 2);               //[se(3) X N+2]  Twists for each BCF + Base + EE in BCF
    matrix *d_eta = zeros(6, numBody + 2);             //[se(3) X N+2]  Twist Rate for each BCF + Base + EE Frame in BCF
    matrix *F = zeros(6, numBody + 2);                 //[se(3) X N+1]  Wrench for each Joint + EE in BCF
    matrix *C = zeros(1, numBody);                     //[se(3) X N]    Actuated Control for each Joint in BCF

    matrix *CoM2CoM = zeros(4, 4);
    //[]     FDM Coefficients for BDF-2
    double c0 = 1.5 / dt;//60
    double c1 = -2 / dt;//-80
    double c2 = .5 / dt;//20


    //Solve fwd flex boundary conditions

    Flex_MB_BCS_params *params = malloc(sizeof(Flex_MB_BCS_params));

    params->robot = robot;

    params->F_ext = matrix_new(F_ext->numRows, F_ext->numCols);
    copyMatrix(F_ext, params->F_ext);
    //params.F_ext = F_ext;
    params->c0 = c0;
    params->c1 = c1;
    params->c2 = c2;
    params->dt = dt;


    params->Theta = matrix_new(Theta->numRows, Theta->numCols);
    copyMatrix(Theta, params->Theta);
    //params.Theta = Theta;

    params->Theta_dot = matrix_new(Theta_dot->numRows, Theta_dot->numCols);
    copyMatrix(Theta_dot, params->Theta_dot);
    //params.Theta_dot = Theta_dot;
    params->Theta_ddot = matrix_new(Theta_DDot->numRows, Theta_DDot->numCols);
    copyMatrix(Theta_DDot_guess, params->Theta_ddot);

    //params.Theta_ddot = Theta_DDot;

    params->C_des = matrix_new(C_des->numRows, C_des->numCols);
    copyMatrix(C_des, params->C_des);
    //params.C_des = C_des;
    params->inv = 1;

    params->F_0 = matrix_new(F_0->numRows, F_0->numCols);
    copyMatrix(F_0, params->F_0);
    //params.F_0 = F_0;

    matrix *JointAcc = matrix_new(Theta_DDot_guess->numRows, 1);
    matrix *tempGuess = matrix_new(Theta_DDot_guess->numRows, 1);
    matrix *tempT = matrix_new(1, 5);
    matrixToFile(matrix_transpose(Theta_DDot_guess, tempT), "fdmFirstSolveInit.csv");
    assert(isnan(robot->objects[1]->object->joint->velocity)==0);
    #if VERBOSE >= 2
    printf("____________theta ddot guess_______________________\n");
    printMatrix(Theta_DDot_guess);
    #endif
    assert(isnan(robot->objects[1]->object->joint->velocity)==0);
    copyMatrix(Theta_DDot_guess, tempGuess);
    int status = find_roots_hybrid(tempGuess, params, 1, TOLERANCE_FWD);


    #if VERBOSE >= 2
    printf("HYBRID DONE, status: %d\n", status);
    #endif
    if (status != 0 ) {
        #if VERBOSE >= 2
        printf("hybrid method failed to converge. Trying levmar\n");
        #endif
        copyMatrix(Theta_DDot_guess, tempGuess);
        status = find_roots_levmarqrt(tempGuess, params, 1, TOLERANCE_FWD);

        if (status != 6) {
            #if VERBOSE >= 2
            printf("levmar method failed to converge trying newton\n");
            #endif
            copyMatrix(Theta_DDot_guess, tempGuess);
            status = find_roots_newton(tempGuess, params, 1, TOLERANCE_FWD);
            if(status != 0){
                printf("ALL FAILED IN FDM_MB_RE FIRST SOLVE");
                #if SOLVER_ERRORS == 1 || SOLVER_ERROR_TOP == 1
                assert(1 == 0);
                #endif

            }
        }
    }
    copyMatrix(tempGuess, JointAcc);
    matrixToFile(matrix_transpose(JointAcc, tempT), "fdmFirstSolve.csv");

    #if VERBOSE >= 2
    printf("_______________FORWARD BCS SOLUTION______________________\n");
    printMatrix(JointAcc);
    printf("___________________________________________\n\n");
    #endif
    eye(g_ref[0]);
    eye(g_act_wrt_prev[0]);

    setSection(eta, 0, 5, 0, 0, zeros(6, 1));
    setSection(d_eta, 0, 5, 0, 0, zeros(6, 1));

    matrix *F_temp = zeros(6, 1);
    Theta = zeros(numBody, 1);
    setSection(Theta, 0, numBody - 1, 0, 0, Theta);

    rigidKin *kin = rigidKinAlloc();
    matrix *F_dist;
    flexDyn *dyn = flexDynAlloc();


    matrix **etaPrev = malloc(sizeof(matrix) * (numBody + 2));
    for (int i = 0; i < numBody + 2; i++) {//todo this could be numFlex I think
        etaPrev[i] = zeros(6, 21);
    }

    matrix **etaPPrev = malloc(sizeof(matrix) * (numBody + 2));
    for (int i = 0; i < numBody + 2; i++) {//todo this could be numFlex I think
        etaPPrev[i] = zeros(6, 21);
    }

    matrix **fPrev = malloc(sizeof(matrix) * (numBody + 2));
    for (int i = 0; i < numBody + 2; i++) {//todo this could be numFlex I think
        fPrev[i] = zeros(6, 21);
    }
    matrix **fPPrev = malloc(sizeof(matrix) * (numBody + 2));
    for (int i = 0; i < numBody + 2; i++) {//todo this could be numFlex I think
        fPPrev[i] = zeros(6, 21);
    }


    matrix *tempR6n1 = matrix_new(6, 1);
    matrix *tempR6n2 = matrix_new(6, 1);
    matrix *tempR6n3 = matrix_new(6, 1);
    matrix *tempR6n4 = matrix_new(6, 1);

    matrix *tempR6n1t = matrix_new(1, 6);

    matrix *temp4x4n1 = matrix_new(4, 4);

    matrix *temp6x6n1 = matrix_new(6, 6);
    matrix *temp6x6n2 = matrix_new(6, 6);

    matrix *temp1 = matrix_new(1,1);
//    int num = 0;
//
//    matrix
//    for(int i = 0; i < (robot->numObjects/2)-1; i++){
//        if(robot->objects[(i*2)+1]->type == 2){
//
//            robot->objects[(2*i)+1]->object->joint->acceleration = JointAcc->data[num];
//            robot->objects[(2*i)+1]->object->joint->velocity += JointAcc->data[num]*dt;
//            assert(isnan(robot->objects[(2*i)+1]->object->joint->velocity)==0);
//            //printf("%f", robot->objects[(2*i)+1]->object->joint->velocity);
//            num++;
//        }
//    }

    matrix *accel_old = zeros(Theta_DDot->numRows,1);
    matrix *vel_old = zeros(Theta_DDot->numRows,1);
    int num = 0;
    for(int i = 0; i <= (robot->numObjects/2)-1; i++){
        if(robot->objects[(2*i)+1]->type == 2){
            accel_old->data[num] = robot->objects[(2*i)+1]->object->joint->acceleration;
            robot->objects[(2*i)+1]->object->joint->acceleration = JointAcc->data[num];

            vel_old->data[num] = robot->objects[(2*i)+1]->object->joint->velocity;
            assert(isnan(robot->objects[(2*i)+1]->object->joint->velocity)==0);
            robot->objects[(2*i)+1]->object->joint->velocity += JointAcc->data[num] * dt;
            num ++;
        }
    }

    matrix* Theta_ddot_old = zeros(Theta_DDot->numRows,1);
    copyMatrix(params->Theta_ddot, Theta_ddot_old);
    copyMatrix(JointAcc, params->Theta_ddot);

    //solve inverse boundary condition
    params->inv = 0;
    matrix *StrGuess = matrix_new(F_0->numRows, 1);
    copyMatrix(F_0, StrGuess);

    #if VERBOSE >= 2
    printf("-------------------fdm 2-----------------------------\n");
    #endif
    matrixToFile(StrGuess, "fdmSecondSolveInit.csv");
    status = find_roots_hybrid(StrGuess, params, 0, TOLERANCE_FWD);
    //printMatrix(StrGuess);
    if (status != 0) {
        #if VERBOSE  >= 1
        printf("hybrid method failed to converge. Trying levmar\n");
        #endif
        copyMatrix(F_0, StrGuess);
        status = find_roots_levmarqrt(StrGuess, params, 0, TOLERANCE_FWD);
        //printMatrix(StrGuess);
        if (status != 6) {
            #if VERBOSE >= 2
            printf("levmar method failed to converge trying newton\n");
            #endif
            copyMatrix(F_0, StrGuess);
            status = find_roots_newton(StrGuess, params, 0, TOLERANCE_FWD);
            if(status != 0){
                #if VERBOSE == 1
                printf("ALL FAILED IN FDM_MB_RE SECOND SOLVE");
                #endif
                #if SOLVER_ERRORS == 1 || SOLVER_ERROR_TOP == 1
                assert(1 == 0);
                #endif
            }
        }
    }
    copyMatrix( Theta_ddot_old, params->Theta_ddot);

    matrixToFile(StrGuess, "fdmSecondSolve.csv");
#if VERBOSE >= 2
    printf("-------------------fdm 2 end-----------------------------\n");
#endif

    for (int i = 1; i < numBody + 2; i++) {
        //printMatrix(C);
        //printf("\n\n");
        rigidJoint *joint = robot->objects[2 * (i - 1) + 1]->object->joint;
        Object *body = robot->objects[2 * i];
        assert(robot->objects[2 * (i - 1) + 1]->type == 2);
        assert(robot->objects[2 * i - 2]->type == 1 || robot->objects[2 * i - 2]->type == 0);


        CoM2CoM = getCoM2CoM(joint, CoM2CoM);
//        printf("CoM2CoM\n");
//        printMatrix(CoM2CoM);
//        printf("g_ref[i - 1]");
//        printMatrix(g_ref[i - 1]);
        zeroMatrix(tempR6n1);

        getSection(eta, 0, 5, i - 1, i - 1, tempR6n1);

        kin = actuateRigidJoint(g_ref[i - 1], CoM2CoM, joint,
                          tempR6n1, getSection(d_eta, 0, 5, i - 1, i - 1, tempR6n2), kin);

        copyMatrix(kin->g_act_wrt_prev, g_act_wrt_prev[i]);
        //g_act_wrt_prev[i] = kin->g_act_wrt_prev;
        copyMatrix(kin->g_cur, g_ref[i]);
        //g_ref[i] = kin->g_cur;

        setSection(eta, 0, 5, i, i, kin->eta);
        setSection(d_eta, 0, 5, i, i, kin->d_eta);

//        printf("gref-1\n");
//        printMatrix(g_ref[i-1]);
//
//        printf("gref\n");
//        printMatrix(g_ref[i]);
        matMult(matrix_transpose(adj(g_act_wrt_prev[i], temp6x6n1), temp6x6n2), F_temp, F_temp);

        if (body->type == 1) {//flexible body
            if (dyn->eta != NULL) {
                matrix_free(dyn->eta);
            }
            dyn->eta = zeros(6, body->object->flex->N);

            if (dyn->f != NULL) {
                matrix_free(dyn->f);
            }
            dyn->f = zeros(6, body->object->flex->N);
            if (i == BC_Start) {//todo double check this +1

                setSection(F, 0, 5, i, i, matMult(body->object->flex->stiff,
                                                  matrix_sub(StrGuess, body->object->flex->F_0, tempR6n1),
                                                  tempR6n1));
            } else {
                setSection(F, 0, 5, i, i, F_temp);
            }

            getSection(F, 0, 5, i, i, tempR6n1);
            matrix_transpose(tempR6n1, tempR6n1t);
            matMult(tempR6n1t, joint->twistR6, temp1);
            setSection(C, 0, 0, i-1, i-1, temp1);

            F_dist = zeros(6, body->object->flex->N);


            flex_dyn(g_ref[i], F_dist, getSection(F, 0, 5, i, i, tempR6n1), body->object->flex,
                     getSection(eta, 0, 5, i, i, tempR6n2), c0, c1, c2, dyn);

            setSection(d_eta, 0, 5, i, i, dyn->d_eta_end);//added this

            //g_ref[i] = dyn->g_end;
            copyMatrix(dyn->g_end, g_ref[i]);
            matMult(body->object->flex->stiff,
                    matrix_sub(getSection(dyn->f, 0, dyn->f->numRows-1, dyn->f->numCols - 1, dyn->f->numCols - 1, tempR6n1),
                               body->object->flex->F_0, tempR6n1), F_temp);


            setSection(eta, 0, eta->numRows-1, i, i,
                       getSection(dyn->eta, 0, 5, dyn->eta->numCols - 1, dyn->eta->numCols - 1, tempR6n1));


            //update history terms AFTER calculations
            copyMatrix(body->object->flex->f_prev, fPPrev[i]);
            //fPPrev[i] = body->object->flex->f_prev;
            copyMatrix(dyn->f, fPrev[i]);
            //fPrev[i] = dyn->f;

            copyMatrix(body->object->flex->eta_prev, etaPPrev[i]);//double che
            //etaPPrev[i] = *body->object->flex->eta_prev;//double check this

            copyMatrix(dyn->eta, etaPrev[i]);
            //etaPrev[i] = *dyn->eta;


        } else if (i > BC_Start) {//rigid bodies



            setSection(F, 0, 5, i, i, F_temp);// [N;Nm] Save Wrench Between i,i-1_th Body @ CoM Expressed in BCF

            if (i < numBody + 1) {

                setSection(C, 0, 0, i - 1, i - 1,
                               matMult(matrix_transpose(getSection(F, 0, 5, i, i, tempR6n1), tempR6n1t),
                                       joint->twistR6, temp1));

            }

            F_temp = matrix_sub(matrix_add(getSection(F, 0, 5, i, i, tempR6n1),
                                           matMult(matMult(matrix_transpose(
                                                   adj_R6(getSection(eta, 0, 5, i, i, tempR6n2), temp6x6n1),
                                                   temp6x6n1),
                                                           body->object->rigid->mass, temp6x6n1),
                                                   getSection(eta, 0, 5, i, i, tempR6n2), tempR6n2), tempR6n2),
                                matMult(body->object->rigid->mass, getSection(d_eta, 0, 5, i, i, tempR6n3),
                                        tempR6n3), F_temp);




            }

    }

    Object *curr_joint;
    Object *curr_body;
    matrix *tempR6t = matrix_new(1,6);

    setSection(F, 0, F->numRows-1, F->numCols - 1, F->numCols - 1, F_ext);
    for(int i = BC_Start; i >= 1; i--) {
        curr_joint = robot->objects[2 * (i - 1)+1];
        curr_body = robot->objects[2 * i ];

        //transpose(Ad(g_act_wrt_prev(:,:,i+1))) * F(:,i) +

        //6x6n1, r6n2, 6x6n2, r6n3, r6n4
        adj(g_act_wrt_prev[i+1], temp6x6n1);
        matrix_transpose(temp6x6n1, temp6x6n1);

        getSection(F, 0,5,i,i, tempR6n2);

        matMult(temp6x6n1, tempR6n2, tempR6n1);

        //first line
        //+
        // ROBOT{2*i-1}.Mass*d_eta(:,i) - transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i);
        //mass
        if(curr_body->type == 1) {//flexible body
            //temp6x6n2 = curr_body->object->flex->mass;
            copyMatrix(curr_body->object->flex->mass, temp6x6n2);
        }else if(curr_body->type == 0){
            copyMatrix(curr_body->object->rigid->mass, temp6x6n2);
            //temp6x6n2 = curr_body->object->rigid->mass;
        }else{
            assert(1 == 0);//this is a bad way to do errors
        }



        getSection(d_eta, 0,5,i,i, tempR6n3);
        matMult(temp6x6n2, tempR6n3, tempR6n3);
        //-
        //tempr6n3
        getSection(eta, 0,5,i,i, tempR6n4);
        adj_R6(tempR6n4, temp6x6n1);
        matrix_transpose(temp6x6n1, temp6x6n1);
        matMult(temp6x6n2, temp6x6n1, temp6x6n1);
        //mass
        getSection(eta, 0,5,i,i, tempR6n4);
        matMult(temp6x6n1, tempR6n4, tempR6n2);


        matrix_sub(tempR6n3, tempR6n2, tempR6n2);//second line
        matrix_add(tempR6n1, tempR6n2, tempR6n1);//full line
        setSection(F, 0,5,i-1,i-1, tempR6n1);



        if(curr_body->type == 1) {//flexible body
            //temp6x6n2 = curr_body->object->flex->mass;
            copyMatrix(curr_body->object->flex->CoM, tempR6n3);
        }else if(curr_body->type == 0){
            copyMatrix(curr_body->object->rigid->CoM, tempR6n3);
            //temp6x6n2 = curr_body->object->rigid->mass;
        }else{
            assert(1 == 0);//this is a bad way to do errors
        }

        //C(:,i-1) = transpose(F(:,i-1)) * Ad(expm3(hat(-ROBOT{2*i-1}.CoM)))*ROBOT{2*(i-1)}.Twist;

        getSection(F, 0,5,i-1,i-1, tempR6n1);
        matrix_transpose(tempR6n1, tempR6t);

        matrix_scalar_mul( tempR6n3, -1, tempR6n3);
        hat_R6(tempR6n3, temp4x4n1);
        expm_SE3(temp4x4n1, temp4x4n1);
        adj(temp4x4n1, temp6x6n1);

        matMult(tempR6t, temp6x6n1, tempR6t);

        copyMatrix(curr_joint->object->joint->twistR6, tempR6n2);

        matMult(tempR6t, tempR6n2, temp1);
        setSection(C, 0,0,i-1,i-1,temp1);
    }

    matrix *Ct = matrix_new(C->numCols, C->numRows);
    matrix_transpose(C, Ct);
    //matrix_free(C);
    for (int i = 1; i < numBody + 2; i++) {
        Object *body = robot->objects[2 * i];
        if (body->type == 1) {

            copyMatrix(etaPrev[i], body->object->flex->eta_prev);
            copyMatrix(etaPPrev[i], body->object->flex->eta_pprev);

            copyMatrix(fPrev[i], body->object->flex->f_prev);
            copyMatrix(fPPrev[i], body->object->flex->f_pprev);
            //body->object->flex->f_prev = fPrev[i];
            //body->object->flex->f_pprev = fPPrev[i];

        }

    }

    num = 0;
    for(int i = 0; i < (robot->numObjects/2)-1; i++){
        if(robot->objects[(i*2)+1]->type == 2){


            robot->objects[(2*i)+1]->object->joint->acceleration = accel_old->data[num];


            robot->objects[(2*i)+1]->object->joint->velocity = vel_old->data[num];
            //printf("%f", robot->objects[(2*i)+1]->object->joint->velocity);
            num++;
        }
    }

    //free(dyn);//todo write free_flexDyn to fully free
    //free(kin);//todo write free_rigidKin to fully free

    FDM_MB_RE_OUT *out = (FDM_MB_RE_OUT *) malloc(sizeof(FDM_MB_RE_OUT));
    out->C = matrix_new(Ct->numRows, Ct->numCols);
    copyMatrix(Ct, out->C);
    //out->C = Ct;
    out->F = matrix_new(F->numRows, F->numCols);
    copyMatrix(F, out->F);
    //out->F = F;
    out->JointAcc = matrix_new(JointAcc->numRows, JointAcc->numCols);
    copyMatrix(JointAcc, out->JointAcc);
    //out->JointAcc = JointAcc;
    //out->robot_new = robot;


    for (int i = 0; i < numBody + 2; i++) {
        matrix_free(g_ref[i]);
        matrix_free(g_act_wrt_prev[i]);
    }

    freeRigidKin(kin);
    //matrix_free(Ct);
    matrix_free(tempR6n1);
    matrix_free(tempR6n2);
    matrix_free(tempR6n3);
    matrix_free(tempR6n4);

    matrix_free(temp6x6n1);
    matrix_free(temp6x6n2);
    matrix_free(tempR6n1t);

    matrix_free(temp4x4n1);
    matrix_free(F_dist);

    matrix_free(d_eta);
    return out;

}




