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
#include <gsl/gsl_deriv.h>
typedef struct rigidBody_s{
    char *name;
    matrix *mass;
    matrix *Transform;//transform from start to end, se3 (6x1)

    //in R6?
    matrix *CoM;//transformation start to COM todo is start J or I?

}rigidBody;


typedef struct flexBody_s{
    char *name;
    matrix *mass;
    matrix *transform;
    matrix *stiff;
    matrix *damping;
    matrix *F_0;//free strain of the continuum under no applied load todo what?
    int N;//number of elements to discretize the continuum todo some of these ints could be uint_8 or 16 to save memory
    double L;//length of the continuum
    matrix *eta_prev;
    matrix *eta_pprev;
    matrix *f_prev;
    matrix *f_pprev;
    matrix *CoM;
}flexBody;


union body_u {
    rigidBody *rigid;
    flexBody *flex;
};
typedef struct body_s {
    uint8_t type;//0 for rigidBody, 1 for flexBody
    union body_u *body;
}Body;
typedef struct rigidJoint_s{
    char *name;
    matrix *twistR6;//twist axis to define the joint column vector R6
    double position; //magnitude of the joint position (is this not basically just an angle? also why is it not just baked into the twist?)
    double velocity;//magnitude of the joint velocity
    double acceleration;//magnitude of the joint acceleration
    double  *limits;//joint limits
    double homepos;//todo this is an angle right? or a magnitude of a twist axis like position?
    Body *parent;//todo make a union for rigid and flex bodies
    Body *child;

}rigidJoint;


/*
 * struct to hold the output of the rigid kinematics
 *
 *
 * g_old [SE3]        transformation from base frame to ith body CoM in RRC (robot refrence configuration)
 *
 *
 * g_oldToCur [SE3]   transformation from ith body CoM to i-1th body CoM in RAC (Robot Actuated Configuration)
 *
 *
 * eta [matrix]       old twist for bdy velocity of ith CoM expressed in the ith CoM BCF (Body coordinate frame)
 *
 *
 * d_eta  [matrix]    time-rate fo cahge of body velocity twists of the ith CoM expressed in the ith CoM BCF (Body coordinate frame about ith CoM)
 */
typedef struct rigidKin_s{

    matrix *g_cur;//transformation from base frame to ith body CoM in RRC (robot refrence configuration)
    matrix *g_act_wrt_prev;// transformation from the ith body CoM BCF to the i-1th body Com bcf in RAC (Robot Actuated Configuration)
    matrix *eta;//twist for bdy velocity of ith CoM expressed in the ith CoM BCF (Body coordinate frame) todo column vector??

    // todo is this not just acceleration? time rate of change of velocity and why is it a column vector??
    matrix *d_eta;//time-rate fo cahge of body velocity twists of the ith CoM expressed in the ith CoM BCF (Body coordinate frame about ith CoM)
}rigidKin;

//[g_end,f,eta,d_eta_end]
typedef struct flexDyn_s{
    matrix *g_end;
    matrix *f;
    matrix *eta;
    matrix *d_eta_end;

}flexDyn;

union object_u {
    char *name;//todo can you have this in a union?

    rigidBody *rigid;
    flexBody *flex;
    rigidJoint *joint;
};

typedef struct object_s {
    uint8_t type;//0 for rigidBody, 1 for flexBody, 2 for rigidJoint
    union object_u *object;
}Object;

//this is the kinematic equations to solve for velocities
rigidKin *actuateRigidJoint(matrix *g_old, matrix *g_oldToCur, rigidJoint *joint, matrix *eta_old, matrix *d_eta_old);

rigidBody *newRigidBody(char *name, matrix *mass, matrix *Transform, matrix *CoM);



rigidJoint *newRigidJoint(char *name, matrix *twistR6, double position, double velocity, double acceleration, double *limits, double homepos, Object *parent, Object *child);
flexBody *newFlexBody(char *name, matrix *mass, matrix *stiff, matrix *damping, matrix *F_0, int N, double L);

typedef matrix* (*Interp_function)(matrix**, double);

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
*/
typedef union ODE_u {
    ODE_type ode;
    step_RK_E_h_type step_RK_E_h;
    COSS_ODE_Dsc_type COSS_ODE_Dsc;
} ODE_function;

matrix *odeFunction(matrix *elem1, matrix *elem2);

matrix *COSS_ODE_Dsc(matrix *y, matrix *y_h, matrix *f_sh, flexBody *Body, double c0, matrix *F_dst);

typedef struct COSS_ODE_OUT_s{
    matrix *eta_s;
    matrix *f_s;

}COSS_ODE_OUT;

char* objToJson(Object *obj);

void freeCOSS_ODE_OUT(COSS_ODE_OUT *out);
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
matrix *step_RK_E_h(matrix *y0, matrix **Y_h, double t0, double h, Interp_function Intrpl, ODE_function odefcn_h, matrix *a, matrix *b, matrix *c, flexBody *body);
ODE_type getODEfunction(matrix *elem1, matrix *elem2);
// eta_prev and f_prev are supposed to be column matrices in R6 x the number of bodies x 2 but I got rid of the 1x so they are 3d

/*
 * Integrate the Cosserat PDE for a Flexible Body to return the states and transformation at its end.
 *
 * DETERMINE:    Configuration, Acceleration @ End & Velocity, Strain over Body
 * GIVEN:        Transformation, Velocity, Wrench BC @ Base, FDM Coeff, Body Definition, Applied Loads
 *
 *     - g_base:     Transformation to the Base Frame of the Body wrt to Reference Frame
 *     - F_dist:     Applied Distributed Wrench on the Body (expressed in Body Frame)
 *     - F_base:     Twist for Body velocity of i_th CoM expressed in i_th CoM BCF
 *     - BODY:       Body Object with Relevant Informaiton
 *     - eta_base:   Velocity Twist of Base Frame
 *     - c0:         FDM Coeff - Current Time Step
 *     - c1:         FDM Coeff - Previous Time Step
 *     - c2:         FDM Coeff - PrePrevious Time Step
 *     - d_eta:      Body Acceleration Twists
 *     - eta(_h):    Body Velocity Twists (Finite Difference Approximation using previous values)
 *     - f(_h):      Body Strain Twists   (Finite Difference Approximation using previous values)
 *     - eta_s:      Velocity Spatial Rate Twists
 *     - f_s:        Strain Spatial Rate Twists
 *     - f_sh:       Strain Spatial Rate Twist (Finite Difference Approximation using previous values)
 *
 * Written by BD Bhai
 * function [g_end,f,eta,d_eta_end] = Flex_Dyn(g_base, F_dist, F_base, BODY, eta_base, c0, c1, c2)
 */
flexDyn *flex_dyn(matrix *g_base, matrix *F_dist, matrix *F_base, flexBody *BODY, matrix *eta_base, double c0, double c1, double c2);






typedef struct robot_s {
    char *name;
    int numObjects;
    Object **objects;
}Robot;

int firstFlex(Robot *robot);

matrix *plotRobotConfig(Robot *robot, matrix *theta, double numStep);

char *jointToJson(rigidJoint *joint);
typedef struct flexJoint_s {//todo this is not implemented yet in the cosserat rod code
}flexJoint;

int getBCStart(Robot *robot);
int getBCEnd(Robot *robot);
void robotFree(Robot *robot);
/*
 *
 * F matrix is the actuation forces
 * C matrix is the constraint
 * v matrix is the body velocities
 * robot_new is the robot copy with updated joint positions
 */
typedef struct IDM_MB_RE_OUT_t{

    //<6x7> matrix in the example code, should be actuation forces
    matrix *F;

    //<1x4> matrix in example code, should be constraint
    matrix *C;

    //<6x7> matrix in example code, should be body velocities
    matrix *v;

    //robot copy with updated joint positions todo should this just update the robot?
    Robot *robot_new;

}IDM_MB_RE_OUT;

void robotToFile(Robot *robot, char *filename);
void addRobotState(Robot *robot, char* filename, int num);
matrix *find_roots_PSO(matrix *InitGuess, Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double c0, double c1, double c2);
matrix* getCoM2CoM(rigidJoint *joint, matrix *CoM2CoM);
//inline docs working?
IDM_MB_RE_OUT *IDM_MB_RE(Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double dt, matrix *x);

matrix *find_roots(matrix *InitGuess, Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double c0, double c1, double c2);
// Define the structure for the parameters to pass to the function
typedef struct {
    matrix *InitGuess;
    Robot *robot;
    matrix *Theta;
    matrix *Theta_dot;
    matrix *Theta_DDot;
    matrix *F_ext;
    double c0;
    double c1;
    double c2;
} Flex_MB_BCS_params;
/*
 * function Error = Flex_MB_BCS(InitGuess, ROBOT, THETA, THETA_DOT, THETA_DDOT, F_ext, c0, c1, c2)
 */
matrix *Flex_MB_BCS(matrix *InitGuess, Robot *robot, matrix F_ext, double c0, double c1, double c2);

matrix *fsolve_idm_mb_re(Robot *robot, matrix *Theta, matrix *Theta_dot, matrix *Theta_DDot, matrix *F_ext, double dt, matrix *x);

#endif //COSSERATROD_RKMK_C_MATHLIB_H
