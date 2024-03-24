//
// Created by Charlie Wadds on 2024-03-12.
//
#include "Matrices.h"
#include "testMatrix.h"
#include <stdio.h>

int testEigenVector(char *filename){


}

int testMatrixLib(){

    printf("test matrix new");
    matrix *test = matrix_new(4,4);

    printf("test matrix zeros");
    matrix *test2 = zeros(4,4);
    printMatrix(test2);


    printf("\ntest determinant\n");
    test->data[0][0] = 1;
    test->data[0][1] = 2;
    test->data[0][2] = 3;
    test->data[0][3] = 4;

    test->data[1][0] = 5;
    test->data[1][1] = 6;
    test->data[1][2] = 7;
    test->data[1][3] = 8;

    test->data[2][0] = 9;
    test->data[2][1] = 10;
    test->data[2][2] = 11;
    test->data[2][3] = 12;

    test->data[3][0] = 13;
    test->data[3][1] = 14;
    test->data[3][2] = 15;
    test->data[3][3] = 16;
    printf("the determinant of the matrix: \b");
    printMatrix(test);
    printf("\n is: %f",Det(test));
    return 1;
}


