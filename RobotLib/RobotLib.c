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
            adj(expm_SE3(new_SE3_T(matrix_scalar_mul(hat_R6(joint->child->CoM)->T,
                                                     -1)))),//transform from joint axis to ith body CoM
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

rigidJoint *newRigidJoint(char *name, matrix *twistR6, double position, int velocity, int acceleration, float *limits,
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

matrix *plotRobotConfig(Robot *robot, double *theta, double numStep) {
    matrix *POS = zeros(3,11);//todo this is a hack, I need to make this dynamic
    matrix *g = eye(4);
    int iii = 1;//num points plotted
    int i_R = 1;

    Object *currObj = (Object *) malloc(sizeof(Object));
    for(int i = 0; i < (robot->numObjects - 2)/2; i++){
        currObj->joint =  robot->objects[(i*2)+1].joint;
        if(1){//todo check for rigid, add flex when implemented


            g = matMult(g, expm_SE3(hat_R6( matrix_scalar_mul(currObj->joint->twistR6, (currObj->joint->homepos + theta[i]))))->T);
            printMatrix(g);
            printf("\n\n\n");
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

matrix *linear_intrpl(matrix **y, float mu){

    return matrix_scalar_mul(matrix_add(y[0], matrix_sub(y[1], y[0])), mu);
}

matrix *COSS_ODE_Dsc(matrix *y, matrix *y_h, matrix *f_sh, flexBody *Body, float c0, matrix *F_dst){


    matrix *f = getSection(y, 0, 5, 0, Body->N - 1);
    matrix *eta = getSection(y, 6,11, 0, Body->N - 1);

    matrix *f_h = getSection(y_h, 0, 5, 0, Body->N - 1);
    matrix *eta_h = getSection(y_h, 6,11, 0, Body->N - 1);


    matrix *f_t = matrix_add(f, matrix_scalar_mul(f_sh, c0));
    matrix *eta_t = matrix_add(eta, matrix_scalar_mul(eta_h, c0));

    matrix *f_s = matrix_add(f, matrix_scalar_mul(Body->damping, c0)) +
            matrix_sub(matrix_add((matMult(Body->mass, eta_t),
            matrix_sub((matMult(matMult(matrix_transpose(adj_R6(f)),Body->mass), eta)),
                       matrix_add(matMult(Body->damping, f_sh),
            matMult(matrix_transpose(adj_R6(f)), (matrix_add(matMult(Body->stiff, matrix_sub(f,Body->F_0->T)), matMult(Body->damping, f_t)) ) ), F_dst)))));
}
flexDyn *flex_dyn(SE3 *g_base, matrix *F_dst, matrix F_base, flexBody *body, matrix *eta_base, matrix **eta_prev, matrix **f_prev, float dt, int LA_SemiDsc, char *LA_ODE, char *LG_ODE, char *Intrp_Fcn){


    matrix **eta = (matrix **)malloc(sizeof(matrix *) * body->N);
    matrix **f = (matrix **)malloc(sizeof(matrix *) * body->N);
    matrix **g = (matrix **)malloc(sizeof(matrix *) * body->N);

    matrix *eta_h = zeros(6, body->N);
    matrix *f_h = zeros(6, body->N);
    for(int i = 0; i < body->N; i++){
        eta[i] = zeros(6,1);
        f[i] = zeros(6,1);
        g[i] = zeros(4,4);



    }

    int offset_sd = 0;

    matrix *c_sd = SD_Load(LA_SemiDsc, dt, &offset_sd);
    //eta_prev is 6x1xbody->Nx?


    for(int i = 1+offset_sd; i < c_sd->numCols; i++){
        //todo this is a mess very slow, only works if eta has 6x1 matrices and there is a solid chance it doesent even work then
        //eta_h = matrix_add(getSection(matrix_transpose(eta_h), 0,5,i,i), getSection(matrix_transpose(eta_prev[i - offset_sd]),0,5,i,i));
        eta_h = matrix_add(eta_h, matrix_scalar_mul(getSection(matrix_transpose(eta_prev[i - offset_sd]),0,5,0,body->N -1), c_sd->data[0][i]));
        f_h = matrix_add(f_h, matrix_scalar_mul(getSection(matrix_transpose(f_prev[i - offset_sd]),0,5,0,body->N -1), c_sd->data[0][i]));
    }




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