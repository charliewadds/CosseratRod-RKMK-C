//
// Created by Charlie Wadds on 2024-03-01.
//

#include "RobotLib.h"
#include <stdio.h>

rigidKin *actuateRigidJoint(SE3 *g_old, SE3 *g_oldToCur, rigidJoint *joint, matrix *eta_old, matrix *d_eta_old) {
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
    joint->twistR6 = matMult(
            adj(expm_SE3(new_SE3_T(matrix_scalar_mul(hat_R6(joint->child->CoM)->T,-1)))),//transform from joint axis to ith body CoM
            joint->twistR6);//redefine joint to now be about child CoM


    SE3 *g_cur = new_SE3_T(matMult_elem(g_old->T, g_oldToCur->T));
    SE3 *g_cur_wrt_prev = new_SE3_T(matrix_inverse(g_oldToCur->T));

    SE3 *g_act_wrt_prev = new_SE3_T(matMult(expm_SE3(
                                             new_SE3_T(
                                                     matrix_scalar_mul(
                                                             matrix_scalar_mul(hat_R6(joint->twistR6)->T, -1),
                                                             joint->position)))->T,
                                     g_cur_wrt_prev->T));//todo this is terrible code, I think I need to fix SE3 to be a matrix not a struct or at least be castable there are so many memory leaks
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

    free_SE3(g_cur_wrt_prev);
    //free_SE3(g_cur);
    //free_SE3(g_act_wrt_prev);
    //matrix_free(eta);
    //matrix_free(d_eta);

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

rigidJoint *newRigidJoint(char *name, matrix *twistR6, double position, double velocity, double acceleration, double *limits,
                          double homepos, rigidBody *parent, rigidBody *child) {
    rigidJoint *joint = (rigidJoint *) malloc(sizeof(rigidJoint));
    joint->name = name;
    joint->twistR6 = twistR6;
    joint->position = position;
    joint->velocity = velocity;
    joint->acceleration = acceleration;
    joint->limits = limits;
    joint->homepos = homepos;
    joint->parent = parent;
    joint->child = child;
    return joint;
}

matrix *plotRobotConfig(Robot *robot, matrix *theta, double numStep) {
    matrix *POS = zeros(3,11);//todo this is a hack, I need to make this dynamic
    matrix *g = eye(4);
    int iii = 1;//num points plotted
    int i_R = 1;

    union object_u *currObj = (union object_u *) malloc(sizeof(Object));
    for(int i = 0; i < (robot->numObjects - 2)/2; i++){
        currObj =  robot->objects[(i*2)+1].object;
        if(1){//todo check for rigid, add flex when implemented


            g = matMult(g, expm_SE3(hat_R6( matrix_scalar_mul(currObj->joint->twistR6, (currObj->joint->homepos + theta->data[i][0]))))->T);
            //printMatrix(g);
            //
            // printf("\n\n\n");
            setSection(POS, 0,2, iii, iii, getSection(g, 0, 2, 3, 3));
            iii++;


            //g = g * expm3(hat(ROBOT{2*i}.Child.Transform));
            g = matMult(g, expm_SE3(hat_R6(currObj->joint->child->Transform))->T);

            setSection(POS, 0,2, iii, iii, getSection(g, 0, 2, 3, 3));
            iii++;

       }
    }

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
matrix *COSS_ODE_Dsc(matrix *y, matrix *y_h, matrix *f_sh, flexBody *Body, float c0, matrix *F_dst){

    matrix *f = getSection(y, 0, 5, 0, Body->N - 1);
    matrix *eta = getSection(y, 6,11, 0, Body->N - 1);

    matrix *f_h = getSection(y_h, 0, 5, 0, Body->N - 1);
    matrix *eta_h = getSection(y_h, 6,11, 0, Body->N - 1);


    matrix *f_t = matrix_add(f, matrix_scalar_mul(f_sh, c0));
    matrix *eta_t = matrix_add(eta, matrix_scalar_mul(eta_h, c0));

//    matrix *f_s = matrix_add(f, matrix_scalar_mul(Body->damping, c0)) +
//            matrix_sub(matrix_add((matMult(Body->mass, eta_t),
//            matrix_sub((matMult(matMult(matrix_transpose(adj_R6(f)),Body->mass), eta)),
//                       matrix_add(matMult(Body->damping, f_sh),
//            matMult(matrix_transpose(adj_R6(f)), (matrix_add(matMult(Body->stiff, matrix_sub(f,Body->F_0->T)), matMult(Body->damping, f_t)) ) ), F_dst)))));
}



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


    //holy shit, this is ugly, hope it works
    out->f_s = matrix_solve(
    matrix_add(K, matrix_scalar_mul(C,c0)),
    //(M*eta_t - (adj(eta)')*M*eta - C*f_sh + (adj(f)')*(K*(f - f_0) + C*f_t) + Fd_ext)
    matrix_add(
    matrix_add(
    matrix_sub(
        matMult(
                matMult(
                    matrix_sub(matMult(M,eta_t), matrix_transpose(adj_R6(eta))),
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

flexDyn *flex_dyn(matrix *g_base, matrix *F_dist, matrix *F_base, flexBody *body, matrix *eta_base, float c0, float c1, float c2){


    matrix *eta = zeros(6, body->N);
    matrix *f = zeros(6, body->N);
    matrix **g = (matrix **)malloc(sizeof(matrix *) * body->N);

    matrix *eta_h = matrix_add(matrix_scalar_mul(body->eta_prev, c1), matrix_scalar_mul(body->eta_pprev, c2));
    matrix *f_h = matrix_add(matrix_scalar_mul(body->f_prev, c1), matrix_scalar_mul(body->f_pprev, c2));

    //set g to <4, 4, numBodies>
    for(int i = 0; i < body->N; i++){
        g[i] = zeros(4,4);
    }





    //todo cosserat rod modle uses different integration functions
    //matrix *c_sd = SD_Load(LA_SemiDsc, dt, &offset_sd);

    /*
     *   Initialize States
     *   f(:,1) = BODY.Stiff \ F_base + BODY.F_0;        %[se(3)]    Assign Initial Strain Twist @ Base of Body
     *   eta(:,1) = eta_base;                            %[se(3)]    Assign Initial Velocity Twist @ Base of Body
     *   g(:,:,1) = g_base;                              %[SE(3)]    Assign Configuration @ Base of Body wrt to Reference Frame
     *   ds = BODY.L / (BODY.N - 1);                     %[m]        Spatial Step Size assumed in Numerical Integration
     */

    setSection(f, 0, 5, 0, 0, matrix_add(matrix_solve(body->stiff,F_base), body->F_0->T));
    setSection(eta, 0, 5, 0, 0, eta_base);
    g[0] = g_base;
    double ds = ((double) body->L) / (body->N - 1);

    //integrate numerically, todo should I use the funcitons from cosserat rods?
    matrix *f_sh = zeros(6,1);
    COSS_ODE_OUT ode = *(COSS_ODE_OUT *) malloc(sizeof(COSS_ODE_OUT));
    for(int i = 1; i < body->N; i++) {
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
                        f_sh, body->stiff, body->damping, body->mass, c0, body->F_0->T,
                        getSection(F_dist, 0,5,i,i));

        setSection(f,0,5,i+1,i+1,
                   (matrix_add(getSection(f,0,5,i,i), matrix_scalar_mul(ode.f_s,ds))));


    }





    return NULL;
}






//return first flexible body or -1 if none
int firstFlex(Robot *robot){

    for(int i = 0; i<robot->numObjects; i++){
        if(robot->objects[i].type == 1){
            return i;
        }
    }
    return -1;

}

//return last flexible body or -1 if none
int lastFlex(Robot *robot){
    int last = -1;
    for(int i = 0; i<robot->numObjects; i++){
        if(robot->objects[i].type == 1){
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
matrix *Flex_MB_BCS(matrix *InitGuess, Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double c0, double c1, double c2){


    int BC_Start = firstFlex(robot);
    int BC_End = lastFlex(robot);
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
    Object *curr_joint = malloc(sizeof(union object_u));
    Object *curr_body = malloc(sizeof(union object_u));
    matrix *CoM2CoM = zeros(6,1);
    matrix *F_dist = zeros(6,1);

    rigidKin *kin = malloc(sizeof(rigidKin));
    for(int i = 0; i< BC_End, i++;){

        curr_joint = &(robot->objects[2 * (i - 1)]);
        curr_joint = &(robot->objects[2 * i - 1]);
        CoM2CoM = matMult(matMult(
        expm_SE3( hat_R6(matrix_sub(curr_joint->object->joint->parent->Transform, curr_joint->object->joint->parent->CoM)))->T,
        expm_SE3( hat_R6(matrix_scalar_mul(curr_joint->object->joint->twistR6, curr_joint->object->joint->homepos)))->T),
        expm_SE3( hat_R6(curr_joint->object->joint->child->CoM))->T);

        //todo double check matrix sizes
        kin = actuateRigidJoint(new_SE3_T(g_ref[i - 1]), new_SE3_T(CoM2CoM), curr_joint->object->joint, getSection(eta, 0,5,0,0), getSection(d_eta, 0,5,0,0));
        g_ref[i] = kin->g_cur->T;
        g_act_wrt_prev[i] = kin->g_act_wrt_prev->T;
        setSection(eta, 0,5,i,i, kin->eta);
        setSection(d_eta, 0,5,i,i, kin->d_eta);

        F_temp = matMult(matrix_transpose(adj(new_SE3_T(g_act_wrt_prev[i]))), F_temp);//todo this should be like multiply accumulate

        if(curr_body->type == 1) {//flexible body
            if(i == BC_Start) {
                //ROBOT{2*i-1}.Stiff * (InitGuess - ROBOT{2*i-1}.F_0);
                setSection(F, 0, 5, i, i, matMult(curr_body->object->flex->stiff,
                                                  matrix_sub(InitGuess, curr_body->object->flex->F_0->T)));
            } else{
                setSection(F,0,5,i,i, F_temp);
            }
            F_dist = zeros(6, curr_body->object->flex->N);





        }else if(i>BC_Start) {
            setSection(F,0,5,i,i, F_temp);//todo this else is not done


        }




        }

    }


    free(kin);//todo make sure this is right and addresses are not referenced elsewhere
    return 0;

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
IDM_MB_RE_OUT *IDM_MB_RE(Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double dt, matrix *InitGuess){

    int numBody = robot->numObjects;
    int BC_Start = firstFlex(robot);

    if(BC_Start == -1){
        //todo not sure what to do here, it might just work?
        printf("No Flexible Body Found\n");
    }


    //todo implement 3d zeros()
    //todo this might not need +2, I dont remember if numBodies includes base and EE
    matrix **g_ref =  malloc(sizeof(matrix) * (numBody+2));           //[SE(3) X N+2]  Transformation to i_th C-BCF from/in base BCF for RRC
    for(int i = 0; i < numBody+2; i++){
        g_ref[i] = zeros(4,4);
    }
    matrix *g_act_wrt_prev = malloc(sizeof(matrix) * (numBody+2));  //[SE(3) X N+2]  Transformation to i-1_th C-BCF from/in i_th BCF for RAC
    for(int i = 0; i < numBody+2; i++){
        g_ref[i] = zeros(4,4);
    }

    matrix *eta = zeros(6,numBody + 2);               //[se(3) X N+2]  Twists for each BCF + Base + EE in BCF
    matrix *d_eta = zeros(6,numBody + 2);             //[se(3) X N+2]  Twist Rate for each BCF + Base + EE Frame in BCF
    matrix *F = zeros(6,numBody + 1);                 //[se(3) X N+1]  Wrench for each Joint + EE in BCF
    matrix *C = zeros(1,numBody);                     //[se(3) X N]    Actuated Control for each Joint in BCF


    //[]     FDM Coefficients for BDF-2
    double c0 = 1.5/dt;
    double c1 = -2/dt;
    double c2 = .5/dt;
    //options = optimset('Display','OFF','TolFun',1e-9);
    //todo need to pass function pointer to fsolve, right now it always uses Flex_MB_BCS
    matrix *InitGuess = fsolve(@(InitGuess)Flex_MB_BCS(InitGuess, ROBOT, THETA, THETA_DOT, ...
    THETA_DDOT, F_ext, c0, c1, c2),InitGuess,options);




    IDM_MB_RE_OUT *out = (IDM_MB_RE_OUT *)malloc(sizeof(IDM_MB_RE_OUT));
    //todo add values here
    return out;
}




//
char *jointToJson(rigidJoint *joint) {
    char *output = malloc(sizeof(char) * 100);
    sprintf(output, "{\"name\":\"%s\",\"twistR6\":[%f,%f,%f,%f,%f,%f],\"position\":%d,\"velocity\":%d,\"acceleration\":%d,\"limits\":[%f,%f],\"homepos\":%d}",
            joint->name, joint->twistR6->data[0][0], joint->twistR6->data[0][1], joint->twistR6->data[0][2],
            joint->twistR6->data[0][3], joint->twistR6->data[0][4], joint->twistR6->data[0][5], joint->position,
            joint->velocity, joint->acceleration, joint->limits[0], joint->limits[1], joint->homepos);
    return output;
}