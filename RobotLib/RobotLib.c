//
// Created by Charlie Wadds on 2024-03-01.
//

#include "RobotLib.h"


rigidKin *actuateRigidJoint(matrix *g_old, matrix *g_oldToCur, rigidJoint *joint, SE3 *eta_old, SE3 *d_eta_old) {
    /*
     - g_cur:          Transformation from Base Frame to i_th Body CoM BCF in RRC
     - g_act_wrt_prev: Transformation from i_th Body CoM BCF to i-1_th Body CoM BCF in RAC
     - eta:            Twist for Body velocity of i_th CoM expressed in i_th CoM BCF
     - d_eta:          Time-Rate of Change of Body Velocity twists of i_th CoM expressed in i_th CoM BCF

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


    matrix *g_cur = matMult_elem(g_old, g_oldToCur);
    matrix *g_cur_wrt_prev = matrix_inverse(g_oldToCur);

    matrix *g_act_wrt_prev = matMult(expm_SE3(
                                             new_SE3_T(
                                                     matrix_scalar_mul(
                                                             matrix_scalar_mul(hat_R6(joint->twistR6)->T, -1),
                                                             joint->position)))->T,
                                     g_cur_wrt_prev);//todo this is terrible code, I think I need to fix SE3 to be a matrix not a struct or at least be castable there are so many memory leaks
    matrix *eta = matrix_add(matMult(adj_R6(g_act_wrt_prev), eta_old->T),
                             matrix_scalar_mul(joint->twistR6, joint->velocity));
    matrix *d_eta = matrix_add(matrix_add(matMult(adj_R6(g_act_wrt_prev), d_eta_old->T), matMult(adj(new_SE3_T(eta)),
                                                                                                 matrix_scalar_mul(
                                                                                                         joint->twistR6,
                                                                                                         joint->velocity))),
                               matrix_scalar_mul(joint->twistR6, joint->acceleration));
    rigidKin *kin = (rigidKin *) malloc(sizeof(rigidKin));
    kin->g_cur = new_SE3_T(g_cur);
    kin->g_act_wrt_prev = new_SE3_T(g_act_wrt_prev);
    kin->eta = new_SE3_T(eta);
    kin->d_eta = new_SE3_T(d_eta);

    matrix_free(g_cur_wrt_prev);
    matrix_free(g_cur);
    matrix_free(g_act_wrt_prev);
    matrix_free(eta);
    matrix_free(d_eta);

    return kin;
}