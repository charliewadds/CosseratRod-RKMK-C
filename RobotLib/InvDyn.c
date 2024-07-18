//
// Created by Charlie Wadds on 2024-07-17.
//

#include "RobotLib.h"


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


    int BC_Start = robot->BC_Start;
    int BC_End = robot->BC_End;
    int numBody = robot->numBody;

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
    matrix *tempR6n4 = matrix_new(6,1);

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


        CoM2CoM = getCoM2CoM(curr_joint->object->joint, CoM2CoM);

        //todo double check matrix sizes



        actuateRigidJoint(g_ref[i-1], CoM2CoM, curr_joint->object->joint, getSection(eta, 0,eta->numRows-1,i-1,i-1, tempR6n1), getSection(d_eta, 0,d_eta->numRows-1,i-1,i-1, tempR6n2), kin);
        //memcpy(g_ref[i], kin->g_cur, sizeof(matrix));
        //g_ref[i] = kin->g_cur;
        copyMatrix(kin->g_cur, g_ref[i]);

        //memcpy(g_act_wrt_prev[i], kin->g_act_wrt_prev, sizeof(matrix));
        //g_act_wrt_prev[i] = kin->g_act_wrt_prev;
        copyMatrix(kin->g_act_wrt_prev, g_act_wrt_prev[i]);

        //assert(hasNan(eta) == 0);
        setSection(eta, 0,eta->numRows-1,i,i, kin->eta);
        //assert(hasNan(eta) == 0);
        setSection(d_eta, 0,d_eta->numRows-1,i,i, kin->d_eta);


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
                setSection(F, 0, 5, i, i, matMult(curr_body->object->flex->stiff,
                                                  matrix_sub(InitGuess, curr_body->object->flex->F_0, tempR6n1), tempR6n1));
            } else{
                setSection(F,0,5,i,i, F_temp);
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
        setSection(d_eta, 0,eta->numRows-1,i,i, kin->d_eta);


    }

//    for(int i = numBody + 1; i >= 0; i--){
//        matrix_free(g_ref[i]);
//        matrix_free(g_act_wrt_prev[i]);
//    }

    setSection(F, 0,5,F->numCols - 1,F->numCols - 1, F_ext); //TODO this needs to be added back in, it is the applied wrench to the end effector



    matrix *bodyMass = matrix_new(6,6);
    for(int i = BC_End+1; i >= numBody ; i--){

        curr_body = robot->objects[2 * i ];
        if(curr_body->type == 1){//flex
            //bodyMass = curr_body->object->flex->mass;
            copyMatrix(curr_body->object->flex->mass, bodyMass);

        }else if(curr_body->type == 0){//rigid
            //bodyMass = curr_body->object->rigid->mass;
            copyMatrix(curr_body->object->rigid->mass, bodyMass);
        }

        //got distracted halfway through this so it could be wrong (it was)
        setSection(F,0,F->numRows-1,i-1,i-1,
                   matrix_sub(
                           matrix_add(
                                   matMult(
                                           matrix_transpose(adj(g_act_wrt_prev[i+1], temp6x6n1), temp6x6n1),
                                           getSection(F,0,F->numRows-1,i,i, tempR6n1)
                                           , tempR6n1),
                                   matMult(bodyMass, getSection(d_eta,0,d_eta->numRows-1,i,i, tempR6n2), tempR6n2), tempR6n2),
                           matMult(matMult(matrix_transpose(adj_R6(getSection(eta,0,eta->numRows-1,i,i, tempR6n3), temp6x6n2), temp6x6n2), bodyMass, temp6x6n2), getSection(eta,0,eta->numRows-1,i,i, tempR6n3), tempR6n3),
                           tempR6n1));

        //F(:,i-1) = transpose(Ad(g_act_wrt_prev(:,:,i+1))) * F(:,i) + ROBOT{2*i-1}.Mass*d_eta(:,i) - transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i);


        //transpose(Ad(g_act_wrt_prev(:,:,i+1)))
//        adj(g_act_wrt_prev[i+1], temp6x6n1);
//        matrix_transpose(temp6x6n1, temp6x6n1);
//        //F(:,i)
//        getSection(F,0,F->numRows-1,i,i, tempR6n1);
//        //transpose(Ad(g_act_wrt_prev(:,:,i+1))) * F(:,i)
//        matMult(temp6x6n1, tempR6n1, tempR6n1);
//
//
//        //ROBOT{2*i-1}.Mass*d_eta(:,i)
//        getSection(F,0,F->numRows-1,i,i, tempR6n2);
//        matMult(bodyMass, tempR6n1, tempR6n2);
//
//
//        //transpose(adj(eta(:,i)))
//        getSection(eta,0,eta->numRows-1,i,i, tempR6n3);
//        adj_R6(tempR6n3, temp6x6n2);
//
//        //transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass
//        matMult(temp6x6n2, bodyMass, temp6x6n2);
//
//        //transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i)
//        matMult(temp6x6n2, tempR6n3, tempR6n3);
//
//
//
//        //transpose(Ad(g_act_wrt_prev(:,:,i+1))) * F(:,i) + ROBOT{2*i-1}.Mass*d_eta(:,i)
//        matrix_add(tempR6n1, tempR6n2, tempR6n1);
//
//        //F(:,i-1) = transpose(Ad(g_act_wrt_prev(:,:,i+1))) * F(:,i) + ROBOT{2*i-1}.Mass*d_eta(:,i) - transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i);
//        matrix_sub(tempR6n1, tempR6n3, tempR6n1);
//
//        setSection(F,0,F->numRows-1,i-1,i-1, tempR6n1);


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

    int numBody = robot->numBody;//todo this should not be a magic number
    int BC_Start = robot->BC_Start;
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
        if (status != 6 && status != 2) {
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
    //matrix *tempT = matrix_new(1, 5);
    matrixToFile(matrix_transpose(InitGuess, tempT), "idmSolve.csv");

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

