//
// Created by Charlie Wadds on 2024-07-04.
//
#ifndef _MATRICES_H_
#include "Matrices.h"
#endif

#ifndef COSSERATROD_RKMK_C_ROBOTLIB_H
#include "RobotLib.h"
#endif


#ifndef COSSERATROD_RKMK_C_SOLVERSV1_H
#define COSSERATROD_RKMK_C_SOLVERSV1_H


typedef matrix* (*BCS_func)(matrix *InitGuess, Flex_MB_BCS_params *params);

int find_roots_hybrid_V1(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, BCS_func f);
int find_roots_levmarqrt_V1(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, BCS_func f);
int find_roots_newton_V1(matrix *InitGuess, Flex_MB_BCS_params *p, int fwd, BCS_func f);
#endif //COSSERATROD_RKMK_C_SOLVERSV1_H
