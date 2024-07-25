//
// Created by Charlie Wadds on 2024-07-17.
//

#include "RobotLib.h"

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


    //int Inv = params->inv;



    int BC_Start = robot->BC_Start;
    //int BC_End = robot->BC_End;
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
        if (status != 6 && status != 2) {
#if VERBOSE >= 2
            printf("\t\tlevmar method failed to converge in F_FLEX trying newton\n");
#endif
            copyMatrix(F_0, tempGuess);
            status = find_roots_newton(tempGuess, params, 0, TOLERANCE_INV);
            if(status != 0){
                printf("ALL FAILED IN F_FLEX_MB_BCS\n");
                #if VERBOSE >= 2
                printf("\tALL FAILED IN F_FLEX_MB_BCS\n");
                #endif


                #if SOLVER_ERRORS == 1
                assert(1 == 0);
                #endif
                //return GSL_EFAILED;

            }//else{
//                printf("\tNewton\n");
//            }
        }//else{
//            printf("\tLevmar\n");
//        }
    }//else{
//        printf("\tHybrid\n");
//    }
    params->inv = 1;
#if VERBOSE >= 2

    printMatrix(tempGuess);
    printf("-------------------F_FLEX END-----------------------------\n");
#endif
    copyMatrix(tempGuess, str_guess);
    copyMatrix(theta_ddot_old, params->Theta_ddot);

    //matrix *tempT = matrix_new(1, 6);
    //matrixToFile(matrix_transpose(str_guess, tempT), "Flex.csv");
    //printf("FUFLEX_MB_BCS SOLVER END\n");
    //printMatrix(str_guess);
    //printf("\n");

    //todo this is the same as in IDM_MB_RE, it might be faster to pass all these as arguments or maybe a struct or something
    matrix **g_ref =  malloc(sizeof(matrix) * (numBody+2));           //[SE(3) X N+2]  Transformation to i_th C-BCF from/in base BCF for RRC
    matrix **g_act_wrt_prev = malloc(sizeof(matrix) * (numBody+2));  //[SE(3) X N+2]  Transformation to i-1_th C-BCF from/in i_th BCF for RAC
    for(int i = 0; i < numBody+2; i++){
        g_ref[i] = zeros(4,4);
        g_act_wrt_prev[i] = zeros(4,4);
    }

    

    matrix *eta = zeros(6,7);      //todo magic number         //[se(3) X N+2]  Twists for each BCF + Base + EE in BCF
    matrix *d_eta = zeros(6,7);        //todo magic number     //[se(3) X N+2]  Twist Rate for each BCF + Base + EE Frame in BCF
    matrix *F = zeros(6,robot->numBody+2);   //todo magic number              //[se(3) X N+1]  Wrench for each Joint + EE in BCF

    matrix *C;
    C = zeros(1, robot->numBody);   //todo magic number


    eye(g_ref[0]);           //[SE(3)]    Base Frame Located @ Base Frame so Identity Transform
    eye(g_act_wrt_prev[0]);  //[SE(3)]    Base Frame Located @ Base Frame so Identity Transform

    matrix *F_temp = zeros(6,1);


    Object *curr_joint;
    Object *curr_body;
    matrix *CoM2CoM = zeros(4,4);
    matrix *F_dist = NULL;
   
    
//    matrix *tempR6n1 = matrix_new(6,1);
//    matrix *tempR6n2 = matrix_new(6,1);
//    matrix *tempR6n3 = matrix_new(6,1);
//    matrix *tempR6n4 = matrix_new(6,1);
//
//    matrix *tempR6t = matrix_new(1,6);
//
//    matrix *temp4x4n1 = matrix_new(4,4);
//
//    matrix *tempC6n1 = matrix_new(1,6);
//    matrix *temp6x6n1 = matrix_new(6,6);
//    matrix *temp6x6n2 = matrix_new(6,6);
//    matrix *temp1 = matrix_new(1,1);


//    rigidKin *kin = rigidKinAlloc();
//    flexDyn *dyn = flexDynAlloc();

    for(int i = 1; i <= numBody+1; i++){

        curr_joint = robot->objects[2 * (i - 1)+1];
        curr_body = robot->objects[2 * i ];


        CoM2CoM = getCoM2CoM(curr_joint->object->joint, CoM2CoM);



        actuateRigidJoint(g_ref[i-1], CoM2CoM, curr_joint->object->joint, getSection(eta, 0,5,i-1,i-1, params->temp->tempR6n1), getSection(d_eta, 0,5,i-1,i-1, params->temp->tempR6n2), params->temp->tempRigidKin);


        copyMatrix(params->temp->tempRigidKin->g_cur, g_ref[i]);

        copyMatrix(params->temp->tempRigidKin->g_act_wrt_prev, g_act_wrt_prev[i]);

        setSection(eta, 0,5,i,i, params->temp->tempRigidKin->eta);
        setSection(d_eta, 0,5,i,i, params->temp->tempRigidKin->d_eta);


        matMult(matrix_transpose(adj(g_act_wrt_prev[i], params->temp->temp6x6n1), params->temp->temp6x6n2), F_temp, F_temp);

        if(curr_body->type == 1) {//flexible body
            if(params->temp->tempFlexDyn->eta != NULL){
                matrix_free(params->temp->tempFlexDyn->eta);
            }
            params->temp->tempFlexDyn->eta = zeros(6, curr_body->object->flex->N);

            if(params->temp->tempFlexDyn->f != NULL){
                matrix_free(params->temp->tempFlexDyn->f);
            }

            params->temp->tempFlexDyn->f = zeros(6, curr_body->object->flex->N);

            if(i == BC_Start ) {
                //ROBOT{2*i-1}.Stiff * (InitGuess - ROBOT{2*i-1}.F_0);

                setSection(F, 0, F->numRows-1, i, i, matMult(curr_body->object->flex->stiff,
                                                             matrix_sub(str_guess, curr_body->object->flex->F_0, params->temp->tempR6n1),params->temp->tempR6n1));

            } else{
                setSection(F,0,F->numRows-1,i,i, F_temp);
            }


            params->temp->tempC6n1 = matrix_transpose(getSection(F, 0, F->numRows-1, i, i, params->temp->tempR6n1), params->temp->tempC6n1);
            matMult(params->temp->tempC6n1, curr_joint->object->joint->twistR6, params->temp->temp1);
            setSection(C, 0,0,i-1,i-1,params->temp->temp1);


            if(F_dist != NULL){
                matrix_free(F_dist);
            }
            F_dist = zeros(6, curr_body->object->flex->N);


            flex_dyn(g_ref[i], F_dist, getSection(F, 0, F->numRows-1, i, i, params->temp->tempR6n1), curr_body->object->flex,
                     getSection(eta, 0, eta->numRows-1, i, i, params->temp->tempR6n2), c0, c1, c2, params->temp->tempFlexDyn);

            setSection(d_eta, 0,d_eta->numRows-1,i,i, params->temp->tempFlexDyn->d_eta_end);
            //memcpy(g_ref[i], dyn->g_end, sizeof(matrix));
            copyMatrix(params->temp->tempFlexDyn->g_end, g_ref[i]);


            matMult(curr_body->object->flex->stiff, matrix_sub(getSection(params->temp->tempFlexDyn->f,0,5,params->temp->tempFlexDyn->f->numCols-1,params->temp->tempFlexDyn->f->numCols-1, params->temp->tempR6n1), curr_body->object->flex->F_0, params->temp->tempR6n1), F_temp);
            setSection(eta,0,eta->numRows-1,i, i, getSection(params->temp->tempFlexDyn->eta,0,5,params->temp->tempFlexDyn->eta->numCols-1, params->temp->tempFlexDyn->eta->numCols-1, params->temp->tempR6n1));


        }else if(i>BC_Start) {//rigid bodies
            setSection(F,0,F->numRows-1,i,i, F_temp);// [N;Nm] Save Wrench Between i,i-1_th Body @ CoM Expressed in BCF

            //setSection(C);
            if(i<numBody+1) {
                params->temp->tempC6n1 = matrix_transpose(getSection(F, 0, F->numRows-1, i, i, params->temp->tempR6n1), params->temp->tempC6n1);
                matMult(params->temp->tempC6n1, curr_joint->object->joint->twistR6, params->temp->temp1);
                setSection(C, 0, 0, i - 1, i - 1, params->temp->temp1);
            }

            /*
             * F_temp = F(:,i) + transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i) - ROBOT{2*i-1}.Mass*d_eta(:,i);
             */
            getSection(F, 0,5,i,i, params->temp->tempR6n1);
            getSection(eta,0,5,i,i, params->temp->tempR6n2);


            adj_R6(params->temp->tempR6n2, params->temp->temp6x6n1);
            matrix_transpose_6x6(params->temp->temp6x6n1, params->temp->temp6x6n1);
            //transpose(adj(eta(:,i))) = params->temp->temp6x6n1

            matMult(curr_body->object->rigid->mass, params->temp->tempR6n2, params->temp->tempR6n3);
            //ROBOT{2*i-1}.Mass*eta(:,i)

            matMult(params->temp->temp6x6n1, params->temp->tempR6n3, params->temp->tempR6n2);
            //transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i)


            matrix_add(params->temp->tempR6n1, params->temp->tempR6n2, params->temp->tempR6n1);
            //F(:,i) + transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i)


            matMult(curr_body->object->rigid->mass, getSection(d_eta,0,5,i,i, params->temp->tempR6n2), params->temp->tempR6n3);
            //ROBOT{2*i-1}.Mass*d_eta(:,i)


            matrix_sub(params->temp->tempR6n1,params->temp->tempR6n3, F_temp);
            //F_temp = F(:,i) + transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i) - ROBOT{2*i-1}.Mass*d_eta(:,i);
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

        adj(g_act_wrt_prev[i+1], params->temp->temp6x6n1);
        matrix_transpose(params->temp->temp6x6n1, params->temp->temp6x6n1);

        getSection(F, 0,5,i,i, params->temp->tempR6n2);

        matMult(params->temp->temp6x6n1, params->temp->tempR6n2, params->temp->tempR6n1);

        if(curr_body->type == 1) {//flexible body
            //params->temp->temp6x6n2 = curr_body->object->flex->mass;
            copyMatrix(curr_body->object->flex->mass, params->temp->temp6x6n2);
        }else if(curr_body->type == 0){
            copyMatrix(curr_body->object->rigid->mass, params->temp->temp6x6n2);
            //params->temp->temp6x6n2 = curr_body->object->rigid->mass;
        }else{
            assert(1 == 0);//this is a bad way to do errors
        }



        getSection(d_eta, 0,d_eta->numRows-1,i,i, params->temp->tempR6n3);
        matMult(params->temp->temp6x6n2, params->temp->tempR6n3, params->temp->tempR6n3);
        //-
        //params->temp->tempR6n3
        getSection(eta, 0,5,i,i, params->temp->tempR6n4);
        adj_R6(params->temp->tempR6n4, params->temp->temp6x6n1);
        matrix_transpose(params->temp->temp6x6n1, params->temp->temp6x6n1);
        matMult(params->temp->temp6x6n2, params->temp->temp6x6n1, params->temp->temp6x6n1);
        //mass
        getSection(eta, 0,5,i,i, params->temp->tempR6n4);
        matMult(params->temp->temp6x6n1, params->temp->tempR6n4, params->temp->tempR6n2);


        matrix_sub(params->temp->tempR6n3, params->temp->tempR6n2, params->temp->tempR6n2);//second line
        matrix_add(params->temp->tempR6n1, params->temp->tempR6n2, params->temp->tempR6n1);//full line
        setSection(F, 0,5,i-1,i-1, params->temp->tempR6n1);



        if(curr_body->type == 1) {//flexible body
            //params->temp->temp6x6n2 = curr_body->object->flex->mass;
            copyMatrix(curr_body->object->flex->CoM, params->temp->tempR6n3);
        }else if(curr_body->type == 0){
            copyMatrix(curr_body->object->rigid->CoM, params->temp->tempR6n3);
            //params->temp->temp6x6n2 = curr_body->object->rigid->mass;
        }else{
            assert(1 == 0);//this is a bad way to do errors
        }

        //C(:,i-1) = transpose(F(:,i-1)) * Ad(expm3(hat(-ROBOT{2*i-1}.CoM)))*ROBOT{2*(i-1)}.Twist;

        getSection(F, 0,5,i-1,i-1, params->temp->tempR6n1);
        matrix_transpose(params->temp->tempR6n1, params->temp->tempR6t);

        matrix_scalar_mul( params->temp->tempR6n3, -1, params->temp->tempR6n3);
        hat_R6(params->temp->tempR6n3, params->temp->temp4x4n1);
        expm_SE3(params->temp->temp4x4n1, params->temp->temp4x4n1);
        adj(params->temp->temp4x4n1, params->temp->temp6x6n1);

        matMult(params->temp->tempR6t, params->temp->temp6x6n1, params->temp->tempR6t);

        copyMatrix(curr_joint->object->joint->twistR6, params->temp->tempR6n2);

        matMult(params->temp->tempR6t, params->temp->tempR6n2, params->temp->temp1);
        setSection(C, 0,0,i-1,i-1,params->temp->temp1);



    }
    matrix *C_inv = matrix_new(C->numCols,1);
    matrix_transpose(C, C_inv);

    matrix_sub(params->C_des, C_inv, C_inv);





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

    for(int i = 0; i < numBody+2; i++){
        matrix_free(g_ref[i]);
        matrix_free(g_act_wrt_prev[i]);
    }

    free(g_ref);
    free(g_act_wrt_prev);
    matrix_free(C_inv);
    matrix_free(eta);
    matrix_free(d_eta);
    matrix_free(F);
    matrix_free(C);
    matrix_free(F_temp);
    matrix_free(CoM2CoM);
    matrix_free(F_ext);
    matrix_free(F_0);
    matrix_free(str_guess);
    matrix_free(accel_old);
    matrix_free(vel_old);
    matrix_free(tempGuess);

    matrix_free(theta);
    matrix_free(theta_dot);
    matrix_free(theta_ddot);
    matrix_free(theta_ddot_old);
    matrix_free(C_des);
    matrix_free(F_dist);





    return GSL_SUCCESS;




}

FDM_MB_RE_OUT *FDM_MB_RE(Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double dt, matrix *C_des, matrix *F_0, matrix *Theta_DDot_guess) {

#if GSL_ERROR_HANDLER == 1
    gsl_set_error_handler(&custom_error_handler);
#endif
    int numBody = robot->numBody;
    int BC_Start = robot->BC_Start;
    //int BC_End = getBCEnd(robot);

    if (BC_Start == -1) {
        //todo not sure what to do here, it might just work?
        printf("No Flexible Body Found\n");
    }


    //todo implement 3d zeros()
    //todo this might not need +2, I dont remember if numBodies includes base and EE
    matrix **g_ref = malloc(sizeof(matrix) * (numBody + 2));           //[SE(3) X N+2]  Transformation to i_th C-BCF from/in base BCF for RRC
    matrix **g_act_wrt_prev = malloc(sizeof(matrix) * (numBody + 2));  //[SE(3) X N+2]  Transformation to i-1_th C-BCF from/in i_th BCF for RAC
    for (int i = 0; i < numBody + 2; i++) {
        g_ref[i] = zeros(4, 4);
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

    BCS_temp *temp = allocTemp_BCS();
    params->temp = temp;

    matrix *JointAcc = matrix_new(Theta_DDot_guess->numRows, 1);
    matrix *tempGuess = matrix_new(Theta_DDot_guess->numRows, 1);
    //matrix *tempT = matrix_new(1, Theta_DDot_guess->numRows);
    //matrixToFile(matrix_transpose(Theta_DDot_guess, tempT), "fdmFirstSolveInit.csv");
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

        if (status != 6 && status != 2) {
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

            }//else{
                //printf("Newton\n");
            //}
        }//else{
         //   printf("Levmar\n");
       // }
    }//else{
     //   printf("Hybrid\n");
    //}
    copyMatrix(tempGuess, JointAcc);
    //matrixToFile(matrix_transpose(JointAcc, tempT), "fdmFirstSolve.csv");

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
    //Theta = zeros(numBody, 1);
    //setSection(Theta, 0, numBody - 1, 0, 0, Theta);

    rigidKin *kin = rigidKinAlloc();
    matrix *F_dist = zeros(6, 21);//todo magic number
    flexDyn *dyn = flexDynAlloc();


    matrix **etaPrev = malloc(sizeof(matrix) * (numBody + 2));
    matrix **etaPPrev = malloc(sizeof(matrix) * (numBody + 2));
    matrix **fPrev = malloc(sizeof(matrix) * (numBody + 2));
    matrix **fPPrev = malloc(sizeof(matrix) * (numBody + 2));
    int N = robot->objects[robot->BC_Start+2]->object->flex->N;
    for (int i = 0; i < numBody + 2; i++) {//todo this assumes consistant discretization and allocates points for non-flexible bodies

        etaPrev[i] = zeros(6, N);
        etaPPrev[i] = zeros(6, N);
        fPrev[i] = zeros(6, N);
        fPPrev[i] = zeros(6, N);
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

    matrix *accel_old = zeros(Theta_DDot->numRows,1);
    matrix *vel_old = zeros(Theta_DDot->numRows,1);
    int num = 0;
    for(int i = 0; i < (robot->numObjects/2)-1; i++){
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
    //matrixToFile(StrGuess, "fdmSecondSolveInit.csv");
    status = find_roots_hybrid(StrGuess, params, 0, TOLERANCE_FWD);
    //printMatrix(StrGuess);
    if (status != 0) {
#if VERBOSE  >= 1
        printf("hybrid method failed to converge. Trying levmar\n");
#endif
        copyMatrix(F_0, StrGuess);
        status = find_roots_levmarqrt(StrGuess, params, 0, TOLERANCE_FWD);
        //printMatrix(StrGuess);
        if (status != 6 && status != 2) {
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
            }//else{
//                printf("Newton\n");
//            }
        }//else{
//            printf("Levmar\n");
//        }
    }//else{
//        printf("Hybrid\n");
//    }
    copyMatrix( Theta_ddot_old, params->Theta_ddot);

    //matrixToFile(StrGuess, "fdmSecondSolve.csv");
#if VERBOSE >= 2
    printf("-------------------fdm 2 end-----------------------------\n");
    printMatrix(StrGuess);
    printf("_______________________________________________________\n");
#endif

    for(int i = 1; i <= numBody+1; i++) {
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

            if(F_dist != NULL){

            }
            zeroMatrix(F_dist);


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
    for(int i = 0; i < (robot->numObjects/2) -1; i++){
        if(robot->objects[(i*2)+1]->type == 2){


            robot->objects[(2*i)+1]->object->joint->acceleration = accel_old->data[num];


            robot->objects[(2*i)+1]->object->joint->velocity = vel_old->data[num];
            //printf("%f", robot->objects[(2*i)+1]->object->joint->velocity);
            num++;
        }
    }

    //free(dyn);//todo write free_flexDyn to fully free
    //free(kin);//todo write free_rigidKin to fully free

    freeFlexDyn(dyn);

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

    for (int i = 0; i < numBody + 2; i++) {//todo this could be numFlex I think
        matrix_free(etaPrev[i]);
        matrix_free(etaPPrev[i]);
        matrix_free(fPrev[i]);
        matrix_free(fPPrev[i]);

    }
    free(etaPrev);
    free(etaPPrev);
    free(fPrev);
    free(fPPrev);

    matrix_free(JointAcc);
    matrix_free(tempGuess);
    matrix_free(accel_old);
    matrix_free(vel_old);
    matrix_free(Theta_ddot_old);
    matrix_free(Ct);


    matrix_free(eta);
    matrix_free(d_eta);
    matrix_free(F);
    matrix_free(C);
    matrix_free(F_temp);


    freeTemp_BCS(temp);
    free(g_ref);
    free(g_act_wrt_prev);
    freeRigidKin(kin);
    matrix_free(tempR6n1);
    matrix_free(tempR6n2);
    matrix_free(tempR6n3);
    matrix_free(tempR6n4);
    matrix_free(temp1);
    matrix_free(temp6x6n1);
    matrix_free(temp6x6n2);
    matrix_free(tempR6n1t);

    matrix_free(CoM2CoM);
    matrix_free(temp4x4n1);
    matrix_free(F_dist);

    return out;

}




