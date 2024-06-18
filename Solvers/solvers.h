//
// Created by Charlie Wadds on 2024-06-18.
//

#ifndef COSSERATROD_RKMK_C_SOLVERS_H
#define COSSERATROD_RKMK_C_SOLVERS_H
#include "Matrices.h"




/*
 * f function of the form f(matrix *x, void *params, matrix *f)
 * n number of variables
 * params parameters of the function
 */
typedef struct multisolve_function_struct
{
    int (* f) (matrix *x, void * params, matrix *f);
    size_t n;
    void * params;
}multisolve_function;






#endif //COSSERATROD_RKMK_C_SOLVERS_H
