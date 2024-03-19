//
// Created by Charlie Wadds on 2024-03-18.
//

#include <string.h>
#include "FDM.h"
#include <stdio.h>

matrix *bdf1(double h_SD){

    matrix *vect = zeros(2,1);
    double a = 1/h_SD;
    vect->data[0][0] = a;
    vect->data[1][0] = a* -1;

    return vect;
}




matrix *SD_Load(int Discretization, double h_SD, int *offset_sd_return){
    if(Discretization == 1){
        if(offset_sd_return != NULL){
            *offset_sd_return = 1;
        }

        return bdf1(h_SD);
    }else{
        printf("Discretization not supported yet\n");
    }
}