//
// Created by Charlie Wadds on 2024-03-01.
//

//#ifndef _MATRICES_H_
//#include "Matrices.h"
//#endif

#ifndef COSSERATROD_RKMK_C_ROBOTLIB_H
#define COSSERATROD_RKMK_C_ROBOTLIB_H
#include "LieGroup.h"



typedef struct rigidBody_s{
    char *name;
    double mass;
    SE3 *Transform;//transform from start to end todo is start J or I?

    //in R6?
    matrix *CoM;//transformation start to COM todo is start J or I?

}rigidBody;


typedef struct flexBody_s{
    char *name;
    double mass;
    matrix *stiff;
    matrix *damping;
    SE3 *F_0;//free strain of the continuum under no applied load todo what?
    int N;//number of elements to discretize the continuum todo some of these ints could be uint_8 or 16 to save memory
    int L;//length of the continuum
}flexBody;

typedef struct rigidJoint_s{
    char *name;
    matrix *twistR6;//twist axis to define the joint column vector R6
    int position; //magnitude of the joint position (is this not basically just an angle? also why is it not just baked into the twist?)
    int velocity;//magnitude of the joint velocity
    int acceleration;//magnitude of the joint acceleration
    float *limits;//joint limits
    int homepos;// todo should this not be in SE3? or is it like position
    rigidBody *parent;//todo this should be either a rigid or flex body not sure how to implement that
    rigidBody *child;//todo same here

}rigidJoint;

typedef struct rigidKin_s{//todo this might not be the best way to do this
    //todo also I have no Idea what any of this means
    SE3 *g_cur;//transformation from base frame to ith body CoM in RRC (robot refrence configuration)
    SE3 *g_act_wrt_prev;// transformation from the ith body CoM BCF to the i-1th body Com bcf in RAC (Robot Actuated Configuration)
    matrix *eta;//twist for bdy velocity of ith CoM expressed in the ith CoM BCF (Body coordinate frame)

    // todo is this not just acceleration? time rate of change of velocity
    matrix *d_eta;//time-rate fo cahge of body velocity twists of the ith CoM expressed in the ith CoM BCF (Body coordinate frame about ith CoM)
}rigidKin;

//this is the kinematic equations to solve for velocities
rigidKin *actuateRigidJoint(SE3 *g_old, SE3 *g_oldToCur, rigidJoint *joint, matrix *eta_old, matrix *d_eta_old);

typedef struct flexJoint_s {//todo this is not implemented yet in the cosserat rod code
}flexJoint;
#endif //COSSERATROD_RKMK_C_MATHLIB_H
