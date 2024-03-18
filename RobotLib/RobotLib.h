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
    matrix *mass;
    matrix *Transform;//transform from start to end

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
    double position; //magnitude of the joint position (is this not basically just an angle? also why is it not just baked into the twist?)
    int velocity;//magnitude of the joint velocity
    int acceleration;//magnitude of the joint acceleration
    float *limits;//joint limits
    float homepos;//todo this is an angle right? or a magnitude of a twist axis like position?
    rigidBody *parent;//todo make a union for rigid and flex bodies
    rigidBody *child;

}rigidJoint;

typedef struct rigidKin_s{

    SE3 *g_cur;//transformation from base frame to ith body CoM in RRC (robot refrence configuration)
    SE3 *g_act_wrt_prev;// transformation from the ith body CoM BCF to the i-1th body Com bcf in RAC (Robot Actuated Configuration)
    matrix *eta;//twist for bdy velocity of ith CoM expressed in the ith CoM BCF (Body coordinate frame) todo column vector??

    // todo is this not just acceleration? time rate of change of velocity and why is it a column vector??
    matrix *d_eta;//time-rate fo cahge of body velocity twists of the ith CoM expressed in the ith CoM BCF (Body coordinate frame about ith CoM)
}rigidKin;

//this is the kinematic equations to solve for velocities
rigidKin *actuateRigidJoint(SE3 *g_old, SE3 *g_oldToCur, rigidJoint *joint, matrix *eta_old, matrix *d_eta_old);

rigidBody *newRigidBody(char *name, matrix *mass, matrix *Transform, matrix *CoM);
rigidJoint *newRigidJoint(char *name, matrix *twistR6, double position, int velocity, int acceleration, float *limits, double homepos, rigidBody *parent, rigidBody *child);
flexBody *newFlexBody(char *name, double mass, matrix *stiff, matrix *damping, SE3 *F_0, int N, int L);

typedef union object_u {
    char *name;
    rigidBody *rigid;
    flexBody *flex;
    rigidJoint *joint;
}Object;

typedef struct robot_s {
    char *name;
    int numObjects;
    Object *objects;


}Robot;

matrix *plotRobotConfig(Robot *robot, double *theta, double numStep);

char *jointToJson(rigidJoint *joint);
typedef struct flexJoint_s {//todo this is not implemented yet in the cosserat rod code
}flexJoint;

#endif //COSSERATROD_RKMK_C_MATHLIB_H
