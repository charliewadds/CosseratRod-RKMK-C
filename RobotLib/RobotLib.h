//
// Created by Charlie Wadds on 2024-03-01.
//

//#ifndef _MATRICES_H_
//#include "Matrices.h"
//#endif

#ifndef COSSERATROD_RKMK_C_ROBOTLIB_H
#define COSSERATROD_RKMK_C_ROBOTLIB_H
#include "LieGroup.h"
#include "FDM.h"








typedef struct rigidBody_s{
    char *name;
    matrix *mass;
    matrix *Transform;//transform from start to end

    //in R6?
    matrix *CoM;//transformation start to COM todo is start J or I?

}rigidBody;


typedef struct flexBody_s{
    char *name;
    matrix *mass;
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


typedef struct flexDyn_s{
    matrix *g;
    matrix *f;
    matrix *eta;
    matrix *d_eta;

}flexDyn;
//this is the kinematic equations to solve for velocities
rigidKin *actuateRigidJoint(SE3 *g_old, SE3 *g_oldToCur, rigidJoint *joint, matrix *eta_old, matrix *d_eta_old);

rigidBody *newRigidBody(char *name, matrix *mass, matrix *Transform, matrix *CoM);
rigidJoint *newRigidJoint(char *name, matrix *twistR6, double position, int velocity, int acceleration, float *limits, double homepos, rigidBody *parent, rigidBody *child);
flexBody *newFlexBody(char *name, double mass, matrix *stiff, matrix *damping, SE3 *F_0, int N, int L);

typedef matrix* (*Interp_function)(matrix**, float);

typedef matrix* (*ODE_type)(matrix*, matrix*);
typedef matrix* (*step_RK_E_h_type)(matrix*, matrix**, float, float, ODE_type, matrix*, matrix*, matrix*);

/*
 * function [y_s] = Coss_ODE_Dsc(y,y_h,f_sh,Body,c0,F_dst)
% Cosserat Model for a Continuum Semi-Discretized from PDE into a Spatial ODE (!! FLEXIBLE ONLY !!)
%
% DETERMINE:    Spatial Derivative of Velocity and Strain Twists
% GIVEN:        Velocity, Strain Twists & Stiffness & Mass & Free Strain & Applied Loading
%
%     - y(_h):      Strain & Velocity Twists (Finite Difference Approximation using previous values)
%     - f_sh:       Strain Spatial Rate Twist (Finite Difference Approximation using previous values)
%     - BODY:       Definition of Flexible Body
%     - c_0:        FDM Coefficient at Current Time Step
%     - F_dst:     Applied Distributed Wrench on the Body (expressed in Body Frame)
%
% Written by BD Bhai
 */
typedef matrix* (*COSS_ODE_Dsc_type)(matrix*, matrix*, matrix*, flexBody*, float, matrix*);
typedef union ODE_u {
    ODE_type ode;
    step_RK_E_h_type step_RK_E_h;
    COSS_ODE_Dsc_type COSS_ODE_Dsc;
} ODE_function;
matrix *odeFunction(matrix *elem1, matrix *elem2);
matrix *COSS_ODE_Dsc(matrix *y, matrix *y_h, matrix *f_sh, flexBody *Body, float c0, matrix *F_dst);
/*Time Stepper for the Runge Kutta Method using an Explicit Integration Scheme
 * DETERMINE:    Spatial Derivative of Velocity and Strain Twists
% GIVEN:        Velocity, Strain Twists & Stiffness & Mass & Free Strain & Applied Loading
%
%     - y1:         Final State
%     - y0:         Initial State
%     - Y_h:        History Terms from Semi-Discretization
%     - y_h:        Interpolation for History Terms Evaluated at a Point
%     - t0:         Initial Time (or whatever you're integrating with respect to really)
%     - h:          Temporal Step Size (or whatever you're integrating with respect to really)
%     - odefcn_h:   ODE function with arguments (time, states, state_h)
%     - odefcn:     ODE function with arguments (time, states)
%     - a:          Weights for RK Integrator
%     - b:          Weights for RK Integrator
%     - c:          Weights for RK Integrator
 step_RK_E_h(y0,Y_h,t0,h,Intrpl,odefcn_h,a,b,c)
 * */
matrix *step_RK_E_h(matrix *y0, matrix **Y_h, float t0, float h, Interp_function Intrpl, ODE_function odefcn_h, matrix *a, matrix *b, matrix *c, flexBody *body);
ODE_type getODEfunction(matrix *elem1, matrix *elem2);
// eta_prev and f_prev are supposed to be column matrices in R6 x the number of bodies x 2 but I got rid of the 1x so they are 3d
flexDyn *flex_dyn(SE3 *g_base, matrix *F_dst, matrix F_base, flexBody *body, matrix *eta_base, matrix **eta_prev, matrix **f_prev, float dt, int LA_SemiDsc, char *LA_ODE, char *LG_ODE, char *Intrp_Fcn);



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
