#include <stdio.h>
#include <assert.h>
#include "RobotLib.h"
#include "Matrices.h"
#include "LieGroup.h"



matrix *randMatrix(int rows, int cols){
    matrix *m = matrix_new(rows, cols);
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            m->data[i][j] = rand() % 10;
        }
    }
    return m;
}

int main() {

    return 0;
}