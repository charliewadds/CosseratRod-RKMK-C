//
// Created by Charlie Wadds on 2024-03-19.
//
#include "FDM.h"

matrix *step_RK_E_h(matrix *y0, matrix **Y_h, float t0, float h, Interp_function Intrpl, ODE_function odefcn_h, matrix *a, matrix *b, matrix *c, flexBody *body){

    matrix **k = (matrix **)malloc(sizeof(matrix *) * b->numCols);
    matrix *y_intrpl = (matrix *)malloc(sizeof(matrix));
    matrix *temp = zeros(y0->numRows, y0->numCols);
    for(int i=0; i < b->numCols; i++){
        k[i] = zeros(y0->numRows, y0->numCols);
    }

    for(int i = 0; i < b->numCols; i++){
        y_intrpl = Intrpl(Y_h, t0 + c->data[0][i] * h);

        for(int j = 0; j< i, j++;){
            matrix_add(temp, matrix_scalar_mul(k[j], a->data[i][j]));
        }

        k[i] = odefcn_h((t0+ c->data[i][0]*h), matrix_add(y0, temp),y, y_intrpl);
    }
}
