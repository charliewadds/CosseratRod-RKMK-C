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


//Error codes (stolen from levmar)
/* O: information regarding the minimization. Set to NULL if don't care
* info[0]= ||e||_2 at initial p.
* info[1-4]=[ ||e||_2, ||J^T e||_inf,  ||Dp||_2, mu/max[J^T J]_ii ], all computed at estimated p.
* info[5]= # iterations,
* info[6]=reason for terminating: 1 - stopped by small gradient J^T e
*                                 2 - stopped by small Dp
*                                 3 - stopped by itmax
*                                 4 - singular matrix. Restart from current p with increased mu
*                                 5 - no further error reduction is possible. Restart with increased mu
*                                 6 - stopped by small ||e||_2
*                                 7 - stopped by invalid (i.e. NaN or Inf) "func" values. This is a user error
* info[7]= # function evaluations
* info[8]= # Jacobian evaluations
* info[9]= # linear systems solved, i.e. # attempts for reducing error
*/

#define V1_SMALL_GRADIENT 1
#define V1_SMALL_DP 2
#define V1_ITMAX 3
#define V1_SINGULAR_MATRIX 4
#define V1_NO_FURTHER_ERROR_REDUCTION 5
#define V1_SMALL_E 6
#define V1_INVALID_FUNC 7

typedef matrix* (*BCS_func)(matrix *InitGuess, Flex_MB_BCS_params *params);

int find_roots_hybrid_V1(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, BCS_func f);
int find_roots_levmarqrt_V1(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, BCS_func f);
int find_roots_newton_V1(matrix *InitGuess, Flex_MB_BCS_params *p, int fwd, BCS_func f);


int fSolver(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, BCS_func f, int solver);
#endif //COSSERATROD_RKMK_C_SOLVERSV1_H
