//
// Created by Charlie Wadds on 2024-02-28.
//

#include <math.h>
#include <assert.h>
#include <printf.h>
#include "LieGroup.h"



//CONSTRUCTORS
//______________________________________________________________________
SO3 *new_SO3(matrix *R){
    SO3 *T = (SO3 *)malloc(sizeof(SO3));
    T->R = R;
    return T;
}


SO3 *new_SO3_zeros(){
    matrix *R = zeros(3,3);
    SO3 *T = (SO3 *)malloc(sizeof(SO3));
    T->R = R;
    return T;
}
void free_SO3(SO3 *T){
    matrix_free(T->R);
    free(T);
}




SE3 *new_SE3(matrix *R, matrix *P){
    SE3 *T = (SE3 *)malloc(sizeof(SE3));
    T->R = new_SO3(R);

    T->P = P;
    T->T = matrix_new(4,4);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            T->T->data[i][j] = R->data[i][j];
        }
    }
    for(int i = 0; i < 3; i++){
        T->T->data[i][3] = P->data[i][0];
    }
    T->T->data[3][0] = 0;
    T->T->data[3][1] = 0;
    T->T->data[3][2] = 0;
    T->T->data[3][3] = 1;
    return T;
}


SE3 *new_SE3_T(matrix *T){
    SE3 *new = (SE3 *)malloc(sizeof(SE3));
    new->R = new_SO3_zeros();
    new->R->R = getSection(T, 0, 2, 0, 2);
    new->P = getSection(T, 0, 2, 3, 3);
    new->T = T;
    return new;
}
SE3 *new_SE3_zeros(){
    SO3 *R = new_SO3_zeros();
    matrix *P = matrix_new(3,1);
    SE3 *T = (SE3 *)malloc(sizeof(SE3));
    T->R = R;
    T->P = P;
    T->T = matrix_new(4,4);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            T->T->data[i][j] = R->R->data[i][j];
        }
    }
    for(int i = 0; i < 3; i++){
        T->T->data[i][3] = P->data[i][0];
    }
    T->T->data[3][0] = 0;
    T->T->data[3][1] = 0;
    T->T->data[3][2] = 0;
    T->T->data[3][3] = 1;
    return T;
}

void free_SE3(SE3 *T){
    free_SO3(T->R);
    matrix_free(T->P);
    matrix_free(T->T);
    free(T);
}


//combine 3x3 rotation matrix and 3x1 position vector into 4x4 transformation matrix
matrix *T_from_PR(matrix* restrict R, matrix* restrict P){
    matrix *T = matrix_new(4,4);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            T->data[i][j] = R->data[i][j];
        }
    }
    for(int i = 0; i < 3; i++){
        T->data[i][3] = P->data[i][0];
    }
    T->data[3][0] = 0;
    T->data[3][1] = 0;
    T->data[3][2] = 0;
    T->data[3][3] = 1;
    return T;
}

matrix *T_from_PR_SO3(SO3 *R, matrix *P){
    matrix *T = matrix_new(4,4);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            T->data[i][j] = R->R->data[i][j];
        }
    }
    for(int i = 0; i < 3; i++){
        T->data[i][3] = P->data[i][0];
    }
    T->data[3][0] = 0;
    T->data[3][1] = 0;
    T->data[3][2] = 0;
    T->data[3][3] = 1;
    return T;
}
//_________________________________________________________________________________________________________


//HAT OPERATORS
//_________________________________________________________________________________________________________
SO3 *hat_R3(matrix *z){
    SO3 *T = new_SO3_zeros();
    T->R = zeros(3,3);
    //printMatrix(T->R);
    T->R->data[0][1] = z->data[2][0] * -1;
    T->R->data[0][2] = z->data[1][0];

    T->R->data[1][0] = z->data[2][0];
    T->R->data[1][2] = z->data[0][0] * -1;

    T->R->data[2][0] = z->data[1][0] * -1;
    T->R->data[2][1] = z->data[0][0];
    return T;
}

SE3 *hat_R6(matrix *z){
    assert(z->numRows == 6);
    assert(z->numCols == 1);

    matrix *T = matrix_new(4,4);
    setSection(T, 0, 2, 0, 2, hat_R3(getSection(z, 3, 5, 0, 0))->R);
    setSection(T, 0, 2, 3, 3, getSection(z, 0, 2, 0, 0));
    setSection(T, 3, 3, 0, 3, zeros(1,4));
    //T->data[3][3] = 1;//todo this is not in matlab, dont I need it?


    //printMatrix(z);
    SE3 *T_hat = new_SE3_T(T);

    //bottom row is all zeros for se3^

    return T_hat;
}

//_________________________________________________________________________________________________________


//UNHAT OPERATORS
//_________________________________________________________________________________________________________

matrix *unhat_SO3(SO3 *zhat){
    matrix *z = matrix_new(3,1);

    z->data[0] = & zhat->R->data[2][1];
    z->data[1] = & zhat->R->data[0][2];
    z->data[2] = & zhat->R->data[1][0];

    return z;
}

matrix *unhat_SE3(SE3 *zhat){
    matrix *z = matrix_new(6,1);


    z->data[0] = zhat->P->data[0];
    z->data[1] = zhat->P->data[1];
    z->data[2] = zhat->P->data[2];

    //todo this would be cleaner if I used unhat_SO3
    z->data[3] = & zhat->R->R->data[2][1];
    z->data[4] = & zhat->R->R->data[0][2];
    z->data[5] = & zhat->R->R->data[1][0];

    return z;
}
//_________________________________________________________________________________________________________

//ADJOINT
//_________________________________________________________________________________________________________
matrix *adj(SE3 *T) {
    matrix *r = matrix_new(6,6);
    //set R, top left 3x3
    setSection(r, 0, 2, 0, 2, T->R->R);

    //set top right 3x3
    setSection(r, 0, 2, 1, 3, matMult(matrix_new(3,3), T->R->R));

    //set bottom left 3x3, zeros
    setSection(r, 1, 3, 0, 2, zeros(3,3));

    //set bottom right 3x3
    setSection(r, 1, 3, 1, 3, T->R->R);

    return r;
}

matrix *adj_R6(matrix *z){
    matrix *r = zeros(6,6);

    matrix *gu = getSection(r, 0, 2, 0, 0);

    //set top left 3x3
    setSection(r, 0, 2, 0, 2, hat_R3(getSection(z, 3, 5, 0, 0))->R);

    //set top right 3x3
    setSection(r, 0, 2, 3, 5, hat_R3(getSection(z, 3, 5, 0, 0))->R);
    return r;
}




//_________________________________________________________________________________________________________

//EXPONENTIAL MAP
//_________________________________________________________________________________________________________

SO3 *expm_SO3(SO3 *m) {

    double mag = norm(unhat_SO3(m));
    if (mag == 0) {
        return new_SO3(eye(3));
    } else {
        return new_SO3(matrix_add(matrix_add(eye(3),
                                             matrix_scalar_mul(elemDiv(m->R, mag), sin(mag))),
                                  matrix_scalar_mul(elemDiv(matrixPow(m->R, 2.0), pow(mag, 2.0)),
                                                    (1 - cos(mag)))));


    }
}



SE3 *expm_SE3(SE3 *m) {
    SO3 *gW = new_SO3(getSection(m->T, 0, 2, 0, 2));
    matrix *gU = getSection(m->T, 0, 2, 3, 3);

    double mag = norm(unhat_SO3(gW));
    matrix *A = eye(3);
    if(mag != 0){

        matrix *k = matrix_scalar_mul(elemDiv(matrixPow(gW->R, 2), ((double) pow(mag,3.0))) , (mag - (double) sin(mag)));
        matrix *f = matrix_scalar_mul(elemDiv(gW->R, pow(mag,2.0)) , (1-cos(mag)));
        A = matrix_add(A,matrix_add(k,f));
    }
    SE3 *result = new_SE3_zeros();
    setSection(result->T, 0, 2, 0, 2, expm_SO3(gW)->R);
    setSection(result->T, 0, 2, 3, 3, matMult(A, gU));
    setSection(result->T , 3, 3, 0, 3, zeros(1,4));
    result->T->data[3][3] = 1;
    return result;
}














