//
// Created by Charlie Wadds on 2024-02-28.
//



#include "LieGroup.h"
#include <stdio.h>
int main(void){

    matrix *m = matrix_new(10,11);
    matrix *m1 = getSection(m, 0,5,0,1);
    printf("[%d, %d]", matrix_shape(m1)[0], matrix_shape(m1)[1]);

}

