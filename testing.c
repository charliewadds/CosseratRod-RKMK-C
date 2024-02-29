//
// Created by Charlie Wadds on 2024-02-28.
//



#include "LieGroup.h"
#include <stdio.h>
int main(void){

    matrix *m = matrix_new(3,3);


    m->data[0][0] = 1;
    m->data[0][1] = 2;
    m->data[0][2] = 3;

    m->data[1][0] = 1;
    m->data[1][1] = 2;
    m->data[1][2] = 3;

    m->data[2][0] = 1;
    m->data[2][1] = 2;
    m->data[2][2] = 3;

    matrix *m1 = matrix_new(3,1);


    m1->data[0][0] = 1;
    m1->data[1][0] = 2;
    m1->data[2][0] = 3;



    printMatrix(m);
    printf("\n");
    printMatrix(m1);
    printf("\n");
    printMatrix(dot(m, m1));
    printf("\n");
    printMatrix(eigenvalues(m, 5));
}

