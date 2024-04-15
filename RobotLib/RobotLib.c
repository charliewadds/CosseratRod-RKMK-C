//
// Created by Charlie Wadds on 2024-03-01.
//

#include "RobotLib.h"
#include <stdio.h>
#include <assert.h>


matrix* getCoM2CoM(rigidJoint *joint, matrix *CoM2CoM){
    matrix *parentCoM = malloc(sizeof(matrix));
    matrix *childCoM = malloc(sizeof(matrix));





    if(joint->parent->type == 0){
        parentCoM = joint->parent->body->rigid->CoM;
    }else if(joint->parent->type == 1){
        parentCoM = joint->parent->body->flex->CoM;
    }else{
        printf("Invalid Body Type (Flex_MB_BCS)\n");
        ///return zeros(6,1);
    }


    if(joint->child->type == 0){
        childCoM = joint->child->body->rigid->CoM;
    }else if(joint->child->type == 1){
        childCoM = joint->child->body->flex->CoM;
    }else{
        printf("Invalid Body Type (Flex_MB_BCS)\n");
        //return zeros(6,1);
    }
    assert(parentCoM->numCols == 1);
    assert(parentCoM->numRows == 6);

    assert(childCoM->numCols == 1);
    assert(childCoM->numRows == 6);



    CoM2CoM = matMult(matMult(
                              //curr_obj
                              expm_SE3( hat_R6(matrix_sub(joint->parent->body->rigid->Transform, parentCoM)))->T,
                              expm_SE3( hat_R6(matrix_scalar_mul(joint->twistR6, joint->homepos)))->T),
                      expm_SE3( hat_R6(childCoM))->T);
    //free(parentCoM);
    //free(childCoM);


    return CoM2CoM;

}


rigidKin *actuateRigidJoint(matrix *g_old, matrix *g_oldToCur, rigidJoint *joint, matrix *eta_old, matrix *d_eta_old) {
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
    matrix *childCoM = malloc(sizeof(matrix));

//    if(joint->parent->type == 0){
//
//        parentCoM = joint->parent->body->rigid->CoM;
//    }else if(joint->parent->type == 1){
//
//        parentCoM = joint->parent->body->flex->CoM;
//
//    }

    if(joint->child->type == 0){

        childCoM = joint->child->body->rigid->CoM;
    }else if(joint->child->type == 1){

        childCoM = joint->child->body->flex->CoM;

    }

    joint->twistR6 = matMult(
            adj(expm_SE3(new_SE3_T(matrix_scalar_mul(hat_R6(childCoM)->T,-1)))),//transform from joint axis to ith body CoM
            joint->twistR6);//redefine joint to now be about child CoM


    SE3 *g_cur = new_SE3_T(matMult(g_old, g_oldToCur));
    SE3 *g_cur_wrt_prev = new_SE3_T(matrix_transpose(matrix_inverse(g_oldToCur)));

    SE3 *g_act_wrt_prev = new_SE3_T(matMult(expm_SE3(new_SE3_T(matrix_scalar_mul(matrix_scalar_mul(hat_R6(joint->twistR6)->T, -1), joint->position)))->T, g_cur_wrt_prev->T));

    matrix *eta = matrix_add(matMult(adj(g_act_wrt_prev), eta_old),
                             matrix_scalar_mul(joint->twistR6, joint->velocity));
    matrix *d_eta = matrix_add(matrix_add(matMult(adj(g_act_wrt_prev), d_eta_old), matMult(adj_R6(eta),
                                                                                                 matrix_scalar_mul(
                                                                                                         joint->twistR6,
                                                                                                         joint->velocity))),
                               matrix_scalar_mul(joint->twistR6, joint->acceleration));
    rigidKin *kin = (rigidKin *) malloc(sizeof(rigidKin));
    kin->g_cur = g_cur;
    kin->g_act_wrt_prev = g_act_wrt_prev;
    kin->eta = eta;
    kin->d_eta = d_eta;

    //free_SE3(g_cur_wrt_prev);
    //free_SE3(g_cur);
    //free_SE3(g_act_wrt_prev);
    //matrix_free(eta);
    //matrix_free(d_eta);
    //free(parentCoM);
    //free(childCoM);
    return kin;
}


rigidBody *newRigidBody(char *name, matrix *mass, matrix *Transform, matrix *CoM) {
    rigidBody *body = (rigidBody *) malloc(sizeof(rigidBody));
    body->name = name;
    body->mass = mass;
    body->Transform = Transform;
    body->CoM = CoM;
    return body;
}

flexBody *newFlexBody(char *name, matrix *mass, matrix *stiff, matrix *damping, matrix *F_0, int N, double L){
    flexBody *body = (flexBody *) malloc(sizeof(flexBody));
    body->name = name;
    body->mass = mass;
    body->transform = zeros(6,1);
    body->stiff = stiff;
    body->damping = damping;
    body->F_0 = F_0;
    body->N = N;
    body->L = L;
    body->CoM = zeros(6,1);
    body->eta_prev = zeros(6,N);
    body->eta_pprev = zeros(6,N);
    body->f_prev = zeros(6,N);
    body->f_pprev = zeros(6,N);
    return body;
}

rigidJoint *newRigidJoint(char *name, matrix *twistR6, double position, double velocity, double acceleration, double *limits,
                          double homepos, Object *parent, Object *child) {
    rigidJoint *joint = (rigidJoint *) malloc(sizeof(rigidJoint));
    joint->name = name;
    joint->twistR6 = twistR6;
    joint->position = position;
    joint->velocity = velocity;
    joint->acceleration = acceleration;
    joint->limits = limits;
    joint->homepos = homepos;

    //todo this whole body union thing is a mess it needs to be redone
    if(parent == NULL) {
        joint->parent = NULL;
    }
    else if(parent->type == 0) {

        joint->parent = malloc(sizeof(struct object_s));
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
        joint->parent = NULL;
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
    matrix *POS = zeros(3,11);//todo this is a hack, I need to make this dynamic
    matrix *g = eye(4);
    int iii = 1;//num points plotted
    //int i_R = 1;

    union object_u *currObj;
    for(int i = 0; i < (robot->numObjects - 2)/2; i++){
        currObj =  robot->objects[(i*2)+1]->object;
        //assert(robot->objects[(i*2)+2]->type == 0 || robot->objects[(i*2)+2]->type == 1);
        if(1){//todo check for rigid, add flex when implemented

            g = matMult(g, expm_SE3(hat_R6( matrix_scalar_mul(currObj->joint->twistR6, (currObj->joint->homepos + theta->data[i][0]))))->T);
            //printMatrix(g);
            //
            // printf("\n\n\n");
            setSection(POS, 0,2, iii, iii, getSection(g, 0, 2, 3, 3));
            iii++;


            //g = g * expm3(hat(ROBOT{2*i}.Child.Transform));
            g = matMult(g, expm_SE3(hat_R6(currObj->joint->child->body->rigid->Transform))->T);

            setSection(POS, 0,2, iii, iii, getSection(g, 0, 2, 3, 3));
            iii++;

       }
    }

    //free(currObj);
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
COSS_ODE_OUT COSS_ODE(matrix *eta, matrix *f, matrix *eta_h, matrix *f_h, matrix *f_sh, matrix *K, matrix *C, matrix *M, double c0, matrix *f_0, matrix *Fd_ext) {

    COSS_ODE_OUT *out = (COSS_ODE_OUT *) malloc(sizeof(COSS_ODE_OUT));

    // Time Discretization (definition of c0 & X_h based on FDM used)
    matrix *f_t = matrix_scalar_mul(matrix_add(f, f_h), c0);     // Local Time Discretization for history in Local Coordinates
    matrix *eta_t = matrix_scalar_mul(matrix_add(eta, eta_h), c0);   // Local Time Discretization for history in Local Coordinates


    //holy shit, this is ugly, hope it works, update, it doesn't
    out->f_s = matrix_solve(
    matrix_add(K, matrix_scalar_mul(C,c0)),
    //(M*eta_t - (adj(eta)')*M*eta - C*f_sh + (adj(f)')*(K*(f - f_0) + C*f_t) + Fd_ext)
    matrix_add(
    matrix_add(
    matrix_sub(
        matMult(
                matMult(
                    matrix_sub_broadcast( matrix_transpose(adj_R6(eta)),matMult(M,eta_t)),
                    M
                ),
                eta
            ),
            matMult(C,f_sh)
            ),matMult(
            matrix_transpose(adj_R6(f)),
            matrix_add(matMult(K, matrix_sub(f, f_0)), matMult(C,f_t))
            )),
    Fd_ext
    ));

    out->eta_s = matrix_add(f_t, matMult(adj_R6(eta),f));


    return *out;
}

void freeCOSS_ODE_OUT(COSS_ODE_OUT *out){
    free(out->eta_s);
    free(out->f_s);
    free(out);
}
flexDyn *flex_dyn(matrix *g_base, matrix *F_dist, matrix *F_base, flexBody *body, matrix *eta_base, double c0, double c1, double c2){


    matrix *eta = zeros(6, body->N);
    matrix *f = zeros(6, body->N);
    matrix **g = (matrix **)malloc(sizeof(matrix *) * body->N);

    matrix *eta_h = matrix_add(matrix_scalar_mul(body->eta_prev, c1), matrix_scalar_mul(body->eta_pprev, c2));
    matrix *f_h = matrix_add(matrix_scalar_mul(body->f_prev, c1), matrix_scalar_mul(body->f_pprev, c2));

    //set g to <4, 4, numBodies>
    for(int i = 0; i < body->N; i++){
        g[i] = zeros(4,4);
    }

    setSection(f, 0, 5, 0, 0, matrix_add(matrix_solve(body->stiff,F_base), body->F_0));
    setSection(eta, 0, 5, 0, 0, eta_base);
    g[0] = g_base;
    double ds = ((double) body->L) / (body->N - 1);

    //integrate numerically, todo should I use the funcitons from cosserat rods?
    matrix *f_sh;
    COSS_ODE_OUT ode;
    for(int i = 0; i < body->N-1; i++) {
        //f_sh = ( c1* (BODY.f_prev(:,i+1) - BODY.f_prev(:,i) ) + ...
        //                 c2*(BODY.f_pprev(:,i+1) - BODY.f_pprev(:,i)) ) / ds;
        //todo should this be i and i-1 or something because matlab is 1 indexed?

        f_sh = elemDiv(
            matrix_add(
                matrix_scalar_mul(matrix_sub(getSection(body->f_prev, 0,5,i+1,i+1), getSection(body->f_prev, 0,5,i,i)),c1),
                matrix_scalar_mul(matrix_sub(getSection(body->f_pprev, 0,5,i+1,i+1), getSection(body->f_pprev, 0,5,i,i)),c2)
            ), ds
        );

        //[f_s,eta_s] = Coss_ODE(eta(:,i), f(:,i), eta_h(:,i), f_h(:,i), f_sh,BODY.Stiff,BODY.Damp,BODY.Mass,c0,BODY.F_0,F_dist(:,i));
        ode = COSS_ODE(getSection(eta,0,5,i,i), getSection(f,0,5,i,i),
                        getSection(eta_h,0,5,i,i), getSection(f_h, 0,5,i,i),
                        f_sh, body->stiff, body->damping, body->mass, c0, body->F_0,
                        getSection(F_dist, 0,5,i,i));

        /*
         * f(:,i+1) = f(:,i) + f_s*ds;                     %[se(3)]    Assuming Euler Integration
         * eta(:,i+1) = eta(:,i) + eta_s*ds;               %[se(3)]    Assuming Euler Integration
         * g(:,:,i+1) = g(:,:,i) * expm(hat(f(:,i))*ds);   %[SE(3)]    Assuming Lie-Euler Geometric Integration
         */
        setSection(f,0,5,i+1,i+1,
                   (matrix_add(getSection(f,0,5,i,i), matrix_scalar_mul(ode.f_s,ds))));

        setSection(eta,0,5,i+1,i+1,
                   (matrix_add(getSection(eta,0,5,i,i), matrix_scalar_mul(ode.eta_s,ds)))
        );

        g[i+1] = matMult(g[i], expm_SE3(new_SE3_T(matrix_scalar_mul(hat_R6(getSection(f,0,5,i,i))->T, ds)))->T);


    }
    //freeCOSS_ODE_OUT(&ode);
    flexDyn *out = (flexDyn *) malloc(sizeof(flexDyn));
    out->g_end = g[body->N-1];
    //d_eta_end = c0*eta(:,end) + eta_h(:,end);
    out->d_eta_end = matrix_add(
            matrix_scalar_mul(getSection(eta, 0,5,body->N, body->N), c0),
            getSection(eta_h, 0,5,body->N, body->N)
    );
    out->f = f;
    out->eta = eta;


    return out;
}






//return first flexible body or -1 if none
int firstFlex(Robot *robot){

    for(int i = 0; i<robot->numObjects; i++){
        if(robot->objects[i]->type == 1){
            return i;
        }
    }
    return -1;

}

//return last flexible body or -1 if none
int lastFlex(Robot *robot){
    int last = -1;
    for(int i = 0; i<robot->numObjects; i++){
        if(robot->objects[i]->type == 1){
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
matrix *Flex_MB_BCS(matrix *InitGuess, Robot *robot, matrix *F_ext, double c0, double c1, double c2){


    int BC_Start = firstFlex(robot)-1;//todo these dont work
    int BC_End = lastFlex(robot)-2;
    int numBody = robot->numObjects;

    if(BC_Start == -1){
        //todo not sure what to do here, it might just work?
        printf("No Flexible Body Found (Flex_MB_BCS)\n");
        return zeros(6,1);
    }

    //todo this is the same as in IDM_MB_RE, it might be faster to pass all these as arguments or maybe a struct or something
    matrix **g_ref =  malloc(sizeof(matrix) * (numBody+2));           //[SE(3) X N+2]  Transformation to i_th C-BCF from/in base BCF for RRC
    for(int i = 0; i < numBody+2; i++){
        g_ref[i] = zeros(4,4);
    }
    matrix **g_act_wrt_prev = malloc(sizeof(matrix) * (numBody+2));  //[SE(3) X N+2]  Transformation to i-1_th C-BCF from/in i_th BCF for RAC
    for(int i = 0; i < numBody+2; i++){
        g_ref[i] = zeros(4,4);
    }

    matrix *eta = zeros(6,numBody + 2);               //[se(3) X N+2]  Twists for each BCF + Base + EE in BCF
    matrix *d_eta = zeros(6,numBody + 2);             //[se(3) X N+2]  Twist Rate for each BCF + Base + EE Frame in BCF
    matrix *F = zeros(6,numBody + 1);                 //[se(3) X N+1]  Wrench for each Joint + EE in BCF

    // Set Initial Conditions
    g_ref[0] = eye(4);           //[SE(3)]    Base Frame Located @ Base Frame so Identity Transform
    g_act_wrt_prev[0] = eye(4);  //[SE(3)]    Base Frame Located @ Base Frame so Identity Transform
    //eta(:,1) = zeros(6,1);           //[se(3)]    Base Frame Stationary so Twist zero todo this does nothing right??
    //d_eta(:,1) = zeros(6,1);         //[se(3)]    Base Frame Stationary so Twist Rate zero todo same here
    matrix *F_temp = zeros(6,1);
    
    //recursive definition of dynamics using Euler-pointcare EOM
    Object *curr_joint ;
    Object *curr_body = malloc(sizeof(union object_u));
    matrix *CoM2CoM = zeros(4,4);
    matrix *F_dist;

    rigidKin *kin;
    flexDyn *dyn;

    //matrix *parentCoM ;
    //matrix *childCoM;
    for(int i = 1; i < BC_End; i++){
        curr_joint = robot->objects[2 * (i - 1) + 1];
        curr_body = robot->objects[2 * i ];


        CoM2CoM = getCoM2CoM(curr_joint->object->joint, CoM2CoM);

        //todo double check matrix sizes
        kin = actuateRigidJoint(g_ref[i-1], CoM2CoM, curr_joint->object->joint, getSection(eta, 0,5,i-1,i-1), getSection(d_eta, 0,5,i-1,i-1));
        g_ref[i] = kin->g_cur->T;
        g_act_wrt_prev[i] = kin->g_act_wrt_prev->T;
        setSection(eta, 0,5,i,i, kin->eta);
        setSection(d_eta, 0,5,i,i, kin->d_eta);

        F_temp = matMult(matrix_transpose(adj(new_SE3_T(g_act_wrt_prev[i]))), F_temp);//todo this should be like multiply accumulate

        if(curr_body->type == 1) {//flexible body
            if(i == BC_Start) {
                //ROBOT{2*i-1}.Stiff * (InitGuess - ROBOT{2*i-1}.F_0);
                setSection(F, 0, 5, i, i, matMult(curr_body->object->flex->stiff,
                                                  matrix_sub(InitGuess, curr_body->object->flex->F_0)));
            } else{
                setSection(F,0,5,i,i, F_temp);
            }
            F_dist = zeros(6, curr_body->object->flex->N);

            //[g_ref(:,:,i),f_cur,eta_cur,d_eta(:,i)] = Flex_Dyn( g_ref(:,:,i), F_dist, F(:,i), ROBOT{2*i-1}, eta(:,i), c0, c1, c2);
            //todo fix this memory leak and also the one in kin
            dyn = flex_dyn(g_ref[i], F_dist, getSection(F, 0, 5, i, i), curr_body->object->flex,
                     getSection(eta, 0, 5, i, i), c0, c1, c2);

            g_ref[i] = dyn->g_end;

            F_temp = matMult(curr_body->object->flex->stiff, matrix_sub(getSection(dyn->f,0,5,dyn->f->numCols-1,dyn->f->numCols-1), curr_body->object->flex->F_0));
            setSection(eta,0,5,eta->numCols, eta->numCols, getSection(dyn->eta,0,5,dyn->eta->numCols, dyn->eta->numCols));


        }else if(i>BC_Start) {//rigid bodies
            setSection(F,0,5,i,i, F_temp);// [N;Nm] Save Wrench Between i,i-1_th Body @ CoM Expressed in BCF

            /*
             * F_temp = F(:,i) + transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i)- ROBOT{2*i-1}.Mass*d_eta(:,i);
             */
            F_temp = matrix_sub(matrix_add(
                    getSection(F, 0,5,i,i),
                    matMult(matrix_transpose(adj_R6(getSection(eta,0,5,i,i))), matMult(curr_body->object->rigid->mass,getSection(eta,0,5,i,i)))
                    ),matMult(curr_body->object->rigid->mass,getSection(eta,0,5,i,i))
            );
        }
    }





    // ALGORITHM FOR LAST ELASTIC BODY TO END OF MANIPULATOR FOR BC LOADS
    for(int i = BC_End + 1; i< 5 +2; i++){//todo fix magic number
        curr_joint = (robot->objects[2 * (i - 1)]);



        CoM2CoM = getCoM2CoM(curr_joint->object->joint, CoM2CoM);


        //% Use Rigid-Body Kinematic Equations to find Velocities, Accelerations and Transformations
        //        [g_ref(:,:,i),g_act_wrt_prev(:,:,i),eta(:,i),d_eta(:,i)] = Rigid_Kin(g_ref(:,:,i-1), CoM2CoM, ROBOT{2*(i-1)}, eta(:,i-1), d_eta(:,i-1));
        kin = actuateRigidJoint(g_ref[i - 1], CoM2CoM, curr_joint->object->joint, getSection(eta, 0,5,0,0), getSection(d_eta, 0,5,0,0));
        g_ref[i] = kin->g_cur->T;
        g_act_wrt_prev[i] = kin->g_act_wrt_prev->T;
        setSection(eta, 0,5,i,i, kin->eta);
        setSection(d_eta, 0,5,i,i, kin->d_eta);

    }

    setSection(F, 0,5,F->numCols,F->numCols, F_ext);

    matrix *bodyMass = malloc(sizeof(matrix));
    for(int i = BC_End +1; i> numBody +1; i--){
        //F(:,i-1) = transpose(Ad(g_act_wrt_prev(:,:,i+1))) * F(:,i) + ...
        //            ROBOT{2*i-1}.Mass*d_eta(:,i) - transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i);     %[N;Nm]     Applied Wrench at i_th CoM  !!EQN FOR RIGID ONLY!!
        //    end                                         %[]         End of {SECTION III} Algorithm
        if(curr_body->type == 1){//flex
            bodyMass = curr_body->object->flex->mass;
        }else if(curr_body->type == 0){//rigid
            bodyMass = curr_body->object->rigid->mass;
        }

        //todo got distracted halfway through this so it could be wrong
        setSection(F,0,5,i-1,i-1,
                   matrix_sub(
                   matrix_add(
                   matMult(
                   matrix_transpose(adj(new_SE3_T(g_act_wrt_prev[i+1]))),
                   getSection(F,0,5,i,i)
                   ),
                   matMult(bodyMass, getSection(d_eta,0,5,i,i))),
                   matMult(matMult(matrix_transpose(adj_R6(getSection(eta,0,5,i,i))), bodyMass), getSection(eta,0,5,i,i))
                   ));


    }
    //free(bodyMass);

    //free(dyn);
    //free(kin);//todo make sure this is right and addresses are not referenced elsewhere

    return matrix_sub(F_temp, getSection(F,0,5,BC_End,BC_End));

}


// Define the function whose roots we want to find
int Flex_MB_BCS_wrapper(const gsl_vector *x, void *params, gsl_vector *f) {
    Flex_MB_BCS_params *p = (Flex_MB_BCS_params *)params;

    // Extracting parameters
    matrix *InitGuess = p->InitGuess;
    Robot *robot = p->robot;
    matrix *F_ext = p->F_ext;
    double c0 = p->c0;
    double c1 = p->c1;
    double c2 = p->c2;

    // Extracting the elements of x
    double x_arr[6];
    for (int i = 0; i < 6; ++i) {
        x_arr[i] = gsl_vector_get(x, i);
    }

    // Convert x to matrix format
    matrix *x_matrix = zeros(6, 1);
    x_matrix->data[0][0] = x_arr[0];
    x_matrix->data[1][0] = x_arr[1];
    x_matrix->data[2][0] = x_arr[2];
    x_matrix->data[3][0] = x_arr[3];
    x_matrix->data[4][0] = x_arr[4];
    x_matrix->data[5][0] = x_arr[5];

    // Call Flex_MB_BCS function
    matrix *result = Flex_MB_BCS(InitGuess, robot, F_ext, c0, c1, c2);

    // Fill f with the residuals
    for (int i = 0; i < 6; ++i) {
        //printf("result: %f\n", result->data[i][0]);
        gsl_vector_set(f, i, result->data[i][0] );
    }

    // Free memory
    //matrix_free(x_matrix);
    //matrix_free(result);

    return GSL_SUCCESS;//todo is this right?
}

// Define the function whose roots we want to find
matrix *Flex_MB_BCS_wrapper_PSO(matrix *x, void *params) {
    Flex_MB_BCS_params *p = (Flex_MB_BCS_params *)params;




    // Extracting parameters
    matrix *InitGuess = p->InitGuess;
    Robot *robot = p->robot;
    matrix *F_ext = p->F_ext;
    double c0 = p->c0;
    double c1 = p->c1;
    double c2 = p->c2;

    return Flex_MB_BCS(x, robot, F_ext, c0, c1, c2);

}

matrix *find_roots_PSO(matrix *InitGuess, Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double c0, double c1, double c2) {
    int numParticles = 10;
    matrix **particlePos = malloc(sizeof(matrix *) * numParticles);
    matrix **particleVect = malloc(sizeof(matrix *) * numParticles);
    Flex_MB_BCS_params params = {InitGuess, robot, Theta, Theta_dot, Theta_DDot, F_ext, c0, c1, c2};

    // Initialize particle positions randomly
    for (int i = 0; i < numParticles; i++) {
        particlePos[i] = matrix_rand(6, 1);
        particleVect[i] = zeros(6, 1);
    }



    double stepSize = 0.0000001; // Step size for PSO

    int maxIter = 100; // Maximum number of iterations

    int bestIndex = 0; // Index of the best particle
    double bestValue = 1000000; // Value of the best particle
    // PSO iteration loop
    matrix error;
    for (int iter = 0; iter < maxIter; iter++) {
        for (int i = 0; i < numParticles; i++) {
            error = *Flex_MB_BCS_wrapper_PSO(particlePos[i], &params);
            if(norm(&error) < bestValue) {
                bestValue = norm(&error);
                bestIndex = i;
                printMatrix(&error);
                printf("\n");
            }else{
                // Update particle position
                particleVect[i] = matrix_sub(particlePos[bestIndex], particlePos[i]);
                particlePos[i] = matrix_add(particlePos[i], matrix_scalar_mul(elemDiv(particleVect[i],matrix_sumSelf(matMult_elem(particleVect[i],particleVect[i]))), stepSize));

            }


        }
    }



    return particlePos[bestIndex]; // Return NULL for now, replace with the best solution found by PSO
}





matrix *find_roots(matrix *InitGuess, Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double c0, double c1, double c2) {
    const gsl_multiroot_fsolver_type *T;
    gsl_multiroot_fsolver *s;

    T = gsl_multiroot_fsolver_dnewton;
    //s = gsl_multiroot_fsolver_allc(T, 6);
    int status;
    size_t iter = 0;

    const size_t n = 6; // Number of variables



    // Set parameters
    Flex_MB_BCS_params params = {InitGuess, robot, Theta, Theta_dot, Theta_DDot, F_ext, c0, c1, c2};
    gsl_multiroot_function f = {&Flex_MB_BCS_wrapper, n, &params};
    //f.params = &params;

    // Define initial guess
    double x_init[6];

    for (int i = 0; i < 6; ++i) {
        x_init[i] = InitGuess->data[i][0];
    }

    gsl_vector_view x_vec = gsl_vector_view_array(x_init, n);
    s = gsl_multiroot_fsolver_alloc(T, n);
    gsl_multiroot_fsolver_set(s, &f, &x_vec.vector);

    do {
        iter++;
        status = gsl_multiroot_fsolver_iterate(s);

        if (status) {
            printf("STATUS: %s\n", gsl_strerror(status));
            break;
        }


        status = gsl_multiroot_test_residual(s->f, 1e-12);

    } while (status == GSL_CONTINUE && iter < 1000);

    // Extract solution
    matrix *solution = zeros(6, 1);
    for (int i = 0; i < 6; ++i) {
        solution->data[i][0] = gsl_vector_get(s->x, i);
    }

    gsl_multiroot_fsolver_free(s);
    return solution;
}

int jacobian_numerical(const gsl_vector *x, void *params, gsl_matrix *J) {
    gsl_vector *f = gsl_vector_alloc(x->size); // Allocate vector for residuals
    gsl_vector *x_plus = gsl_vector_alloc(x->size);
    gsl_vector *x_minus = gsl_vector_alloc(x->size);

    // Compute f(x)
    Flex_MB_BCS_wrapper(x, params, f);

    // Compute Jacobian using finite differences
    for (size_t j = 0; j < x->size; j++) {
        double x_j = gsl_vector_get(x, j);
        double h = 1e-5; // Step size for finite differences

        gsl_vector_memcpy(x_plus, x);
        gsl_vector_set(x_plus, j, x_j + h);
        Flex_MB_BCS_wrapper(x_plus, params, f);

        gsl_vector_memcpy(x_minus, x);
        gsl_vector_set(x_minus, j, x_j - h);
        Flex_MB_BCS_wrapper(x_minus, params, f);

        gsl_vector_sub(x_plus, x_minus);
        gsl_vector_scale(x_plus, 1.0 / (2 * h));

        gsl_matrix_set_col(J, j, x_plus);
    }

    gsl_vector_free(f);
    gsl_vector_free(x_plus);
    gsl_vector_free(x_minus);

    return GSL_SUCCESS;
}

int fdf_function(const gsl_vector *x, void *params, gsl_vector *f, gsl_matrix *J) {
    // Compute the residuals (objectives) using the provided objective function
    Flex_MB_BCS_wrapper(x, params, f);

    // Compute the Jacobian numerically using finite differences
    jacobian_numerical(x, params, J);

    return GSL_SUCCESS;
}

matrix *find_roots_deriv(matrix *InitGuess, Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double c0, double c1, double c2) {
    const gsl_multiroot_fdfsolver_type *T;
    gsl_multiroot_fdfsolver *s;

    //T = gsl_multiroot_fsolver_hybrid;
    //s = gsl_multiroot_fsolver_allc(T, 6);
    int status;
    size_t iter = 0;

    const size_t n = 6; // Number of variables




    //f.params = &params;

    // Define initial guess
    double x_init[6];

    for (int i = 0; i < 6; ++i) {
        x_init[i] = InitGuess->data[i][0];
    }
    double abserror;
    gsl_vector_view x_vec = gsl_vector_view_array(x_init, n);

    gsl_matrix *J = gsl_matrix_alloc(6, 6);
    Flex_MB_BCS_params params = {InitGuess, robot, Theta, Theta_dot, Theta_DDot, F_ext, c0, c1, c2};
    jacobian_numerical(&x_vec.vector, &params, J);
    // Set parameters

    gsl_multiroot_function_fdf f = {&Flex_MB_BCS_wrapper, &jacobian_numerical, &fdf_function,n, &params};


    T = gsl_multiroot_fdfsolver_gnewton;
    s = gsl_multiroot_fdfsolver_alloc(T, 6);
    gsl_multiroot_fdfsolver_set(s, &f, &x_vec.vector);//todo check this cast


    do {
        iter++;
        status = gsl_multiroot_fdfsolver_iterate(s);
        printf("Iteration %zu:\n", iter);
        for (int i = 0; i < 6; i++) {
            printf("x[%d] = %.15f\n", i, gsl_vector_get(s->x, i));
        }
        if (status) {
            printf("STATUS: %s\n", gsl_strerror(status));
            break;
        }


        status = gsl_multiroot_test_residual(s->f, 1e-12);
    } while (status == GSL_CONTINUE && iter < 1000);

    // Extract solution
    matrix *solution = zeros(6, 1);
    for (int i = 0; i < 6; ++i) {
        solution->data[i][0] = gsl_vector_get(s->x, i);
    }

    gsl_multiroot_fdfsolver_free(s);
    return solution;
}

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
    int BC_Start = firstFlex(robot) - 1;

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
    matrix *F = zeros(6, numBody + 1);                 //[se(3) X N+1]  Wrench for each Joint + EE in BCF
    matrix *C = zeros(1, numBody);                     //[se(3) X N]    Actuated Control for each Joint in BCF

    matrix *CoM2CoM = zeros(4, 4);
    //[]     FDM Coefficients for BDF-2
    double c0 = 1.5 / dt;
    double c1 = -2 / dt;
    double c2 = .5 / dt;
    //options = optimset('Display','OFF','TolFun',1e-9);
    //todo need to pass function pointer to fsolve, right now it always uses Flex_MB_BCS
    //todo does this need to find ALL roots or just the one 'nearest' to the initial guess?
    //matrix *InitGuess = fsolve(@(InitGuess)Flex_MB_BCS(InitGuess, ROBOT, THETA, THETA_DOT, ...
    //THETA_DDOT, F_ext, c0, c1, c2),InitGuess,options);
    printf("INIT_GUESS pre\n");

    printMatrix(matrix_sub(ones(6,1),InitGuess));
    InitGuess = find_roots_deriv(InitGuess, robot, Theta, Theta_dot, Theta_DDot, F_ext, c0, c1, c2);
    printf("\nINIT_GUESS post\n");
    //printMatrix(Theta);
    printMatrix(matrix_sub(ones(6,1), InitGuess));
    //printf("ans");
    //printMatrix(Flex_MB_BCS(InitGuess, robot, Theta, Theta_dot, Theta_DDot, F_ext, c0, c1, c2));
    g_ref[0] = eye(4);
    g_act_wrt_prev[0] = eye(4);
    setSection(eta, 0, 5, 0, 0, zeros(6, 1));
    setSection(d_eta, 0, 5, 0, 0, zeros(6, 1));

    matrix *F_temp = zeros(6, 1);
    Theta = zeros(numBody, 1);
    setSection(Theta, 0, numBody - 1, 0, 0, Theta);

    rigidKin *kin ;
    matrix *F_dist ;
    flexDyn *dyn ;

    for (int i = 2; i < numBody + 2; i++) {
        rigidJoint *joint = robot->objects[2 * (i - 1) - 1]->object->joint;
        Object *body = robot->objects[2 * i - 2];
        assert(robot->objects[2 * (i - 1) - 1]->type == 2);
        assert(robot->objects[2 * i - 2]->type == 1 || robot->objects[2 * i - 2]->type == 0);

        CoM2CoM = getCoM2CoM(joint, CoM2CoM);

        kin = actuateRigidJoint(g_ref[i - 1], CoM2CoM, joint,
                                getSection(eta, 0, 5, i - 1, i - 1), getSection(d_eta, 0, 5, i - 1, i - 1));
        g_act_wrt_prev[i] = kin->g_act_wrt_prev->T;
        g_ref[i] = kin->g_cur->T;

        setSection(eta, 0, 5, i, i, kin->eta);
        setSection(d_eta, 0, 5, i, i, kin->d_eta);

        F_temp = matMult(matrix_transpose(adj(new_SE3_T(g_act_wrt_prev[i]))), F_temp);

        if (body->type == 1) {//flexible body
            if (i == BC_Start) {
                setSection(F, 0, 5, i, i, matMult(body->object->flex->stiff,
                                                  matrix_sub(InitGuess, body->object->flex->F_0)));
            } else {
                setSection(F, 0, 5, i, i, F_temp);
            }
            F_dist = zeros(6, body->object->flex->N);
            dyn = flex_dyn(g_ref[i], F_dist, getSection(F, 0, 5, i, i), body->object->flex,
                           getSection(eta, 0, 5, i, i), c0, c1, c2);

            F_temp = matMult(body->object->flex->stiff,
                             matrix_sub(getSection(dyn->f, 0, 5, dyn->f->numCols - 1, dyn->f->numCols - 1),
                                        body->object->flex->F_0));


            setSection(eta, 0, 5, eta->numCols, eta->numCols,
                       getSection(dyn->eta, 0, 5, dyn->eta->numCols, dyn->eta->numCols));


            body->object->flex->f_pprev = body->object->flex->f_prev;
            body->object->flex->f_prev = dyn->f;

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

        if (BC_Start < numBody) {
            matrix *objMass = malloc(sizeof(matrix));
            matrix *objCoM = malloc(sizeof(matrix));


            for (int i = BC_Start; i > 2; i--) {
                rigidJoint *joint = robot->objects[2 * (i - 1) - 1]->object->joint;
                Object *body = robot->objects[2 * i - 2];

                if (body->type == 1) {
                    objMass = body->object->flex->mass;
                    objCoM = body->object->flex->CoM;
                } else if (body->type == 0) {
                    objMass = body->object->rigid->mass;
                    objCoM = body->object->rigid->CoM;
                }
                setSection(F, 0, 5, i - 1, i - 1,

                           matrix_add(
                                   matMult(matrix_transpose(adj(new_SE3_T(g_act_wrt_prev[i + 1]))), getSection(F, 0, 5, i, i)),
                                   matrix_sub(matMult(objMass, getSection(d_eta, 0, 5, i, i)),
                                              matMult(matrix_transpose(adj_R6(getSection(eta, 0, 5, i, i))),
                                                      matMult(objMass, getSection(eta, 0, 5, i, i))))
                           ));
                if(i < numBody+2) {

                    setSection(C, 0, C->numRows - 1, i - 1, i - 1,
                               matrix_outerProduct(matrix_transpose(getSection(F, 0, 5, i, i)), joint->twistR6));
                }
                }
            //matrix_free(objMass);
            //matrix_free(objCoM);
        }

        C = matrix_transpose(C);

        //free(dyn);//todo write free_flexDyn to fully free
        //free(kin);//todo write free_rigidKin to fully free

        IDM_MB_RE_OUT *out = (IDM_MB_RE_OUT *) malloc(sizeof(IDM_MB_RE_OUT));
        out->C = C;
        out->F = F;
        out->v = eta;
        out->robot_new = robot;

        return out;

}



