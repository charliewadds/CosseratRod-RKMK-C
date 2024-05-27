//
// Created by Charlie Wadds on 2024-02-28.
//

#include <math.h>
#include <assert.h>
#include <string.h>
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
    SO3 *T = (SO3 *) malloc(sizeof(SO3));
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
    getSection(T, 0, 2, 0, 2, new->R->R);
    getSection(T, 0, 2, 3, 3, new->P);
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

matrix *hat_R3(matrix *z, matrix *result){
    assert(z->numRows == 3);
    assert(z->numCols == 1);

    //matrix *T = zeros(3,3);

    result->data[0][1] = z->data[2][0] * -1;
    result->data[0][2] = z->data[1][0];

    result->data[1][0] = z->data[2][0];
    result->data[1][2] = z->data[0][0] * -1;

    result->data[2][0] = z->data[1][0] * -1;
    result->data[2][1] = z->data[0][0];
    return result;
}

matrix *hat_R6(matrix *z, matrix *result){
    assert(z->numRows == 6);
    assert(z->numCols == 1);

    assert(result->numRows == 4);
    assert(result->numCols == 4);

    matrix *temp = matrix_new(3,3);
    matrix *temp3x1 = matrix_new(3,1);
    setSection(result, 0, 2, 0, 2, hat_R3(getSection(z, 3, 5, 0, 0, temp3x1), temp));
    setSection(result, 0, 2, 3, 3, getSection(z, 0, 2, 0, 0, temp3x1));
    //getSetSection(z, result,0,2,0,0,0,2,3,5);
    setSection(result, 3, 3, 0, 3, zeros(1,4));//todo convert to zeroSection using memset
    //T->data[3][3] = 1;//todo this is not in matlab, dont I need it?

    matrix_free(temp);
    matrix_free(temp3x1);
    return result;
}


matrix *hat_R6_chain(matrix *z){
    assert(z->numRows == 6);
    assert(z->numCols == 1);

    matrix *result = matrix_new(4,4);
    matrix *temp = matrix_new(3,3);
    matrix *temp3x1 = matrix_new(3,1);
    setSection(result, 0, 2, 0, 2, hat_R3(getSection(z, 3, 5, 0, 0, temp3x1), temp));
    setSection(result, 0, 2, 3, 3, getSection(z, 0, 2, 0, 0, temp3x1));
    setSection(result, 3, 3, 0, 3, zeros(1,4));//todo convert to zeroSection using memset
    //T->data[3][3] = 1;//todo this is not in matlab, dont I need it?

    matrix_free(temp);
    matrix_free(z);
    return result;
}

//_________________________________________________________________________________________________________


//UNHAT OPERATORS
//_________________________________________________________________________________________________________

matrix *unhat_SO3(matrix *zhat, matrix *result){
    assert(zhat->numRows == 3);
    assert(zhat->numCols == 3);

    assert(result->numRows == 3);
    assert(result->numCols == 1);

    //matrix *z = matrix_new(3,1);

    //result->data[0] = &zhat->data[2][1];
    memcpy(result->data[0], &zhat->data[2][1], sizeof(double) * 1);

    //result->data[1] = & zhat->data[0][2];
    memcpy(result->data[1], &zhat->data[0][2], sizeof(double) * 1);

    //result->data[2] = & zhat->data[1][0];
    memcpy(result->data[2], &zhat->data[1][0], sizeof(double) * 1);

    return result;
}

matrix *unhat_SE3(matrix *zhat, matrix *result){
    //matrix *z = matrix_new(6,1);
    assert(result->numRows == 6);
    assert(result->numCols == 1);


    result->data[0] = zhat->data[0];
    result->data[1] = zhat->data[1];
    result->data[2] = zhat->data[2];

    //todo this would be cleaner if I used unhat_SO3
    result->data[3] = & zhat->data[2][1];
    result->data[4] = & zhat->data[0][2];
    result->data[5] = & zhat->data[1][0];

    return result;
}
//_________________________________________________________________________________________________________

//ADJOINT
//_________________________________________________________________________________________________________

matrix *adj(matrix *T, matrix *result) {
    assert(T->numRows == 4);
    assert(T->numCols == 4);

    assert(result->numRows == 6);
    assert(result->numCols == 6);

    zeroMatrix(result); //todo is this necessary? also it could be faster
    matrix *r = matrix_new(3,3);
    getSection(T, 0, 2, 0, 2, r);

    matrix *p = matrix_new(3,1);
    getSection(T, 0, 2, 3, 3, p);

    //matrix *out = zeros(6,6);

    matrix *temp = matrix_new(3,3);

    setSection(result, 0, 2, 0, 2, r);
    setSection(result, 0, 2, 3, 5, matMult( hat_R3(p, temp),r, temp));

    setSection(result, 3, 5, 3, 5, r);
    setSection(result, 3, 5, 0, 2, zeros(3,3));//todo does this make sense?

    matrix_free(temp);
    return result;
}

matrix *adj_chain(matrix *T) {
    assert(T->numRows == 4);
    assert(T->numCols == 4);

    matrix *result = matrix_new(6,6);

    matrix *r = matrix_new(3,3);
    getSection(T, 0, 2, 0, 2, r);

    matrix *p = matrix_new(3,1);
    getSection(T, 0, 2, 3, 3, p);

    //matrix *out = zeros(6,6);

    matrix *temp = matrix_new(3,3);

    setSection(result, 0, 2, 0, 2, r);
    setSection(result, 0, 2, 3, 5, matMult( hat_R3(p, temp),r, temp));

    setSection(result, 3, 5, 3, 5, r);
    setSection(result, 3, 5, 0, 2, zeros(3,3));//todo does this make sense?

    matrix_free(T);
    return result;
}

matrix *adj_R6(matrix *z, matrix *result){
    assert(z->numRows == 6);
    assert(z->numCols == 1);

    assert(result->numRows == 6);
    assert(result->numCols == 6);

    matrix *gu = matrix_new(3,1);
    getSection(z, 0, 2, 0, 0, gu);

    matrix *gw = matrix_new(3,1);
    getSection(z, 3, 5, 0, 0, gw);

    //matrix *r = zeros(6,6);
    matrix *temp = matrix_new(3,3);
    setSection(result, 0, 2, 0, 2, hat_R3(gw, temp));
    setSection(result, 0, 2, 3, 5, hat_R3(gu, temp));
    setSection(result, 3, 5, 3, 5, hat_R3(gw, temp));

    matrix_free(gw);
    matrix_free(gu);
    matrix_free(temp);
    return result;
}




//_________________________________________________________________________________________________________

//EXPONENTIAL MAP
//_________________________________________________________________________________________________________

matrix *expm_SO3(matrix *m, matrix *result){
    assert(m->numRows == 3);
    assert(m->numCols == 3);

    assert(result->numRows == 3);
    assert(result->numCols == 3);

    matrix *temp = matrix_new(3,1);
    double mag = norm(unhat_SO3(m, temp));

    matrix *temp_3x3n1 = matrix_new(3,3);
    matrix *temp_3x3n2 = matrix_new(3,3);


    if (mag == 0) {
        eye(result);
    } else {
        matrix_add(
                matrix_add(
                        eye(result),matrix_scalar_mul(elemDiv(m, mag, temp_3x3n1), sin(mag), temp_3x3n1)
                              , temp_3x3n1),
                                  matrix_scalar_mul(
                                          elemDiv(
                                                  matrixPow(m, 2, temp_3x3n2), pow(mag, 2.0),
                                                  temp_3x3n2
                                                  ),
                                                  (1 - cos(mag))
                                          , temp_3x3n2)
                        , result);


    }

    matrix_free(temp);
    matrix_free(temp_3x3n1);
    matrix_free(temp_3x3n2);
    return result;
}



matrix *expm_SE3(matrix *m, matrix *result) {
    assert(m->numRows == 4);
    assert(m->numCols == 4);

    assert(result->numRows == 4);
    assert(result->numCols == 4);

    matrix *gW = matrix_new(3,3);
    getSection(m, 0, 2, 0, 2, gW);

    matrix *gU = matrix_new(3,1);
    getSection(m, 0, 2, 3, 3, gU);

    matrix *temp3x1 = matrix_new(3,1);
    matrix *temp3x3 = matrix_new(3,3);
    double mag = norm(unhat_SO3(gW, temp3x1));

    matrix *A = eye(matrix_new(3,3));

    if(mag != 0){
        matrix *k = matrix_new(3,3);
        matrix_scalar_mul(elemDiv(matrixPow(gW, 2, temp3x3), ((double) pow(mag,3.0)), temp3x3) , (mag - (double) sin(mag)), k);

        matrix *f = matrix_new(3,3);
        matrix_scalar_mul(elemDiv(gW, pow(mag,2.0), temp3x3) , (1-cos(mag)), f);

        matrix_add3(A,k,f, A);

        matrix_free(k);
        matrix_free(f);
    }
    zeroMatrix(result);
    setSection(result, 0, 2, 0, 2, expm_SO3(gW, temp3x3));
    setSection(result, 0, 2, 3, 3, matMult(A, gU, temp3x3));
    //matrix *temp1x4 = zeros(1,4);
    //setSection(result , 3, 3, 0, 3, temp1x4);
    result->data[3][3] = 1;

    //matrix_free()
    matrix_free(gW);
    matrix_free(gU);
    matrix_free(temp3x1);
    matrix_free(temp3x3);// todo make sure there is not residual memory leak
    //matrix
    matrix_free(A);

    return result;
}

matrix *expm_SE3_chain(matrix *m) {
    assert(m->numRows == 4);
    assert(m->numCols == 4);


    matrix *gW = matrix_new(3,3);
    getSection(m, 0, 2, 0, 2, gW);

    matrix *gU = matrix_new(3,1);
    getSection(m, 0, 2, 3, 3, gU);

    matrix *temp3x3 = matrix_new(3,3);
    matrix *temp3x1 = matrix_new(3,1);
    double mag = norm(unhat_SO3(gW, temp3x1));

    matrix *A = eye(matrix_new(3,3));

    if(mag != 0){
        matrix *k = matrix_new(3,3);
        matrix_scalar_mul(elemDiv(matrixPow(gW, 2, temp3x3), ((double) pow(mag,3.0)), temp3x3) , (mag - (double) sin(mag)), k);

        matrix *f = matrix_new(3,3);
        matrix_scalar_mul(elemDiv(gW, pow(mag,2.0), temp3x3) , (1-cos(mag)), f);

        matrix_add3(A,k,f, A);
    }
    //matrix *result = zeros(4,4);
    setSection(m, 0, 2, 0, 2, expm_SO3(gW, temp3x3));
    setSection(m, 0, 2, 3, 3, matMult(A, gU, temp3x3));
    setSection(m , 3, 3, 0, 3, zeros(1,4));
    m->data[3][3] = 1;


    matrix_free(gW);
    matrix_free(gU);
    matrix_free(temp3x3);
    matrix_free(A);

    return m;
}














