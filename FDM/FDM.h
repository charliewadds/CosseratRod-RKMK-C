//
// Created by Charlie Wadds on 2024-03-18.
//

#ifndef COSSERATROD_RKMK_C_FDM_H
#define COSSERATROD_RKMK_C_FDM_H
#include "LieGroup.h"
#include "RobotLib.h"//todo could redefine structs so I dont need to import the whole library


#define BDF1 1
#define BDF2 2
#define CentDiff 3

#define RK4_a {{0, 0, 0, 0}, {0.5, 0, 0, 0}, {0, 0.5, 0, 0}, {0, 0, 1, 0}}
#define RK4_b = {{1/6}, {1/3}, {1/3}, {1/6}}
#define RK4_c = {{0}, {0.5}, {0.5}, {1}}

#define RK2_a = {{0, 0}, {0.5, 0}}
#define RK2_b = {{0.5}, {0.5}}
#define RK2_c = {{0}, {1}}

#define Imp_mid_a = {{0.5}}//todo there has to be a better way to do this
#define Imp_mid_b = {{1}}
#define Imp_mid_c = {{0.5}}



matrix *bdf1(double h_SD);

matrix *SD_Load(int Discretization, double h_SD, int *offset_sd_return);

matrix *step_RK_E_h(matrix *y0, matrix **Y_h, float t0, float h, Interp_function Intrpl, ODE_function odefcn_h, matrix *a, matrix *b, matrix *c, flexBody *body);

#endif //COSSERATROD_RKMK_C_FDM_H
