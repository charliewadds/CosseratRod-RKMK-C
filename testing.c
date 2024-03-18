//
// Created by Charlie Wadds on 2024-02-28.
//

// todo modern robotics book
//todo add precision macro

#include "RobotLib.h"

#include <stdio.h>
int main(void){
    /*
     *  1.0000   -0.0000    0.0000    0.0000
        0.0000    0.0000   -1.0000   -0.6500
        0.0000    1.0000    0.0000    0.5000
         0         0         0    1.0000

    */
    matrix *g_old = eye(4);
    g_old->data[1][2] = -1;
    g_old->data[1][3] = -0.65;
    g_old->data[2][1] = 1;
    g_old->data[2][3] = 0.5;
    printf("G_old\n");
    printMatrix(g_old);

    /*
_____________\n
    1.0000         0         0         0
         0    1.0000         0         0
         0         0    1.0000    0.2500
         0         0         0    1.0000
*/
    matrix *g_oldToCur = eye(4);
    g_oldToCur->data[2][3] = 0.25;
    printf("\nG_oldToCur\n");
    printMatrix(g_oldToCur);

    /*

____________\n
        Name: 'Joint_4'
        Type: 'RIGID'
       Twist: [6x1 double]
    Position: -9.0270e-04
         Vel: -0.0120
       Accel: -0.0865
       Limit: [-3.1416 3.1416]
     HomePos: 0
      Parent: [1x1 struct]
       Child: [1x1 struct]

       */
    rigidBody *child = (rigidBody *)malloc(sizeof(rigidBody));
    child->name = "Child";
    child->mass = matrix_scalar_mul(eye(6), 1);
    matrix *Trans = matrix_new(6,1);
    Trans->data[0][2] = 1;
    matrix *transform = eye(4);
    transform->data[2][0] = 0.25;

    child->Transform = new_SE3_T(transform);
    child->CoM = elemDiv(Trans, 2);

    rigidJoint *joint = (rigidJoint *)malloc(sizeof(rigidJoint));
    joint->name = "Joint_4";
    matrix *twist = matrix_new(6,1);
    twist->data[0][4] = 1;
    joint->twistR6 = twist;
    joint->position = -0.0009027;
    joint->velocity = -0.0120;
    joint->acceleration = -0.0865;
    joint->limits = (float *)malloc(sizeof(float) * 2);
    joint->limits[0] = -3.1416;
    joint->limits[1] = 3.1416;
    joint->homepos = 0;
    joint->parent = NULL;
    joint->child = child;


    /*


____________\n
    0.0133
    0.0056
    0.0000
   -0.0086
    0.0257
    0.0000
*/
    matrix *eta_old = matrix_new(6,1);
    eta_old->data[0][0] = 0.0133;
    eta_old->data[1][0] = 0.0056;
    eta_old->data[2][0] = 0.0000;
    eta_old->data[3][0] = -0.0086;
    eta_old->data[4][0] = 0.0257;
    eta_old->data[5][0] = 0.0000;
    printf("\nEta_old\n");
    printMatrix(eta_old);
    /*

________________\n
    0.0997
    0.0421
    0.0001
   -0.0648
    0.1914
    0.0001
*/
    matrix *d_eta_old = matrix_new(6,1);
    d_eta_old->data[0][0] = 0.0997;
    d_eta_old->data[1][0] = 0.0421;
    d_eta_old->data[2][0] = 0.0001;
    d_eta_old->data[3][0] = -0.0648;
    d_eta_old->data[4][0] = 0.1914;
    d_eta_old->data[5][0] = 0.0001;
    printf("\nD_Eta_old\n");
    printMatrix(d_eta_old);


    rigidKin *kin = actuateRigidJoint(new_SE3_T(g_old), new_SE3_T(g_oldToCur), joint, eta_old, d_eta_old);
    printf("g_cur\n");
    printMatrix(kin->g_cur->T);
    printf("g_act_wrt_prev\n");
    printMatrix(kin->g_act_wrt_prev->T);

    printf("D_eta\n");
    printMatrix(kin->d_eta);
    printf("Eta\n");
    printMatrix(kin->eta);




    /*

end
    1.0000   -0.0000    0.0000    0.0000
    0.0000    0.0000   -1.0000   -0.9000
    0.0000    1.0000    0.0000    0.5000
         0         0         0    1.0000


    1.0000         0         0         0
         0    1.0000   -0.0009    0.0002
         0    0.0009    1.0000   -0.2500
         0         0         0    1.0000


    0.0197
    0.0077
    0.0000
   -0.0206
    0.0257
    0.0000


    0.1475
    0.0583
    0.0002
   -0.1513
    0.1914
    0.0006
     */




    return 0;




}

