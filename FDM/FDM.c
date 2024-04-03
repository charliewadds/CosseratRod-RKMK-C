//
// Created by Charlie Wadds on 2024-03-19.
//
#include "FDM.h"


//Time Steppper for the Runge Kutta Method using an Explicit Integration Scheme
matrix *step_RK_E_h(matrix *y0, matrix **Y_h, float t0, float h, Interp_function Intrpl, ODE_function odefcn_h, matrix *a, matrix *b, matrix *c, flexBody *body){

    matrix **k = (matrix **)malloc(sizeof(matrix *) * b->numCols);

    //not sure why this interpolates between y and intetrator weights (also what are integrator weights)
    matrix *y_intrpl = (matrix *)malloc(sizeof(matrix));

    //progress from initial state I think this is delta y
    matrix *temp = zeros(y0->numRows, y0->numCols);
    for(int i=0; i < b->numCols; i++){
        k[i] = zeros(y0->numRows, y0->numCols);
    }

    for(int i = 0; i < b->numCols; i++){
        y_intrpl = Intrpl(Y_h, t0 + c->data[0][i] * h);

        for(int j = 0; j< i, j++;){
            matrix_add(temp, matrix_scalar_mul(k[j], a->data[i][j]));
        }
        //y_h = (y_intrpl = Intrpl(Y_h, c(i));)
        //c0 = t0 + c(i)*h
        //y = y0 + temp(progress from initial state)
        k[i] = odefcn_h();
    }
}
