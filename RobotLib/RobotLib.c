//
// Created by Charlie Wadds on 2024-03-01.
//
#include "RobotLib.h"

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
    matrix *tempR6n1 = matrix_new(6,1);

    matrix_scalar_mul(childCoM, -1, tempR6n1);
    hat_R6(tempR6n1, temp4x4n1);
    //matrix_scalar_mul(temp4x4n1,-1, temp4x4n1);
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


    matrix *g_act_wrt_prev = matrix_new(4,4);
    matMult(expm_SE3(matrix_scalar_mul(matrix_scalar_mul(hat_R6(newTwist, temp4x4n1), -1, temp4x4n1), joint->position, temp4x4n1), temp4x4n1), g_cur_wrt_prev, g_act_wrt_prev);


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

    matrix *POS = zeros(3,200);//todo this is a hack, I need to make this dynamic
    matrix *g = matrix_new(4,4);
    eye(g);

    int iii = 1;//num points plotted
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

            getSetSection(g, POS, 0, 2, 3, 3, 0, 2, iii, iii);

            iii++;

        }
        else if(currObj->joint->child->type == 1){


            matMult(g, expm_SE3(hat_R6( matrix_scalar_mul(currObj->joint->twistR6, (currObj->joint->homepos + theta->data[(i * theta->numCols)]), temp6n1), temp4x4n1), temp4x4n1), g);

            double ds = currObj->joint->child->body->flex->L / currObj->joint->child->body->flex->N;

            for(int j = 0; j < (currObj->joint->child->body->flex->N * numStep); j++){
                int index = ceil((j+1)/numStep)-1;

                temp6n1 = getSection(currObj->joint->child->body->flex->f_prev, 0, 5, index, index, temp6n1);
                expm_SE3(hat_R6(matrix_scalar_mul(temp6n1, ds/numStep, temp6n1), temp4x4n1), temp4x4n1);
                g = matMult(g, temp4x4n1, g);
                getSetSection(g, POS, 0, 2, 3, 3, 0, 2, iii, iii);
                iii ++;

            }
        }
    }

    //free(currObj);
    matrix_free(g);
    //matrix_free(POS);
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




    copyMatrix(g[body->N-1], result->g_end);


    matrix_add(
            matrix_scalar_mul(getSection(result->eta, 0,5,result->eta->numCols - 1, result->eta->numCols -1, tempR6n1), c0, tempR6n1),
            getSection(eta_h, 0,5,eta_h->numCols-1, eta_h->numCols-1, tempR6n2),
            result->d_eta_end);
    //matrix_free(result->f)
    //result->f = f;
    //result->eta = eta;

    //freeCOSS_ODE_OUT(&ode);

    freeCOSS_ODE_OUT(ode);
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


void custom_error_handler(const char *reason, const char *file, int line, int gsl_errno) {
    fprintf(stderr, "GSL Error: %s:%d: %s (code %d)\n", file, line, reason, gsl_errno);

}


