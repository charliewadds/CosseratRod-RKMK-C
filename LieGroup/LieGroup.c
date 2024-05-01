//
// Created by Charlie Wadds on 2024-02-28.
//

#include <math.h>
#include <assert.h>
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
matrix *hat_R3(matrix *z){
    assert(z->numRows == 3);
    assert(z->numCols == 1);

    matrix *T = zeros(3,3);

    T->data[0][1] = z->data[2][0] * -1;
    T->data[0][2] = z->data[1][0];

    T->data[1][0] = z->data[2][0];
    T->data[1][2] = z->data[0][0] * -1;

    T->data[2][0] = z->data[1][0] * -1;
    T->data[2][1] = z->data[0][0];
    return T;
}

matrix *hat_R6(matrix *z){
    assert(z->numRows == 6);
    assert(z->numCols == 1);

    matrix *T = matrix_new(4,4);
    setSection(T, 0, 2, 0, 2, hat_R3(getSection(z, 3, 5, 0, 0)));
    setSection(T, 0, 2, 3, 3, getSection(z, 0, 2, 0, 0));
    setSection(T, 3, 3, 0, 3, zeros(1,4));
    //T->data[3][3] = 1;//todo this is not in matlab, dont I need it?

    return T;
}

//_________________________________________________________________________________________________________


//UNHAT OPERATORS
//_________________________________________________________________________________________________________

matrix *unhat_SO3(matrix *zhat){
    assert(zhat->numRows == 3);
    assert(zhat->numCols == 3);

    matrix *z = matrix_new(3,1);

    z->data[0] = & zhat->data[2][1];
    z->data[1] = & zhat->data[0][2];
    z->data[2] = & zhat->data[1][0];

    return z;
}

matrix *unhat_SE3(matrix *zhat){
    matrix *z = matrix_new(6,1);


    z->data[0] = zhat->data[0];
    z->data[1] = zhat->data[1];
    z->data[2] = zhat->data[2];

    //todo this would be cleaner if I used unhat_SO3
    z->data[3] = & zhat->data[2][1];
    z->data[4] = & zhat->data[0][2];
    z->data[5] = & zhat->data[1][0];

    return z;
}
//_________________________________________________________________________________________________________

//ADJOINT
//_________________________________________________________________________________________________________
matrix *adj(matrix *T) {
    assert(T->numRows == 4);
    assert(T->numCols == 4);

    matrix *r = getSection(T, 0, 2, 0, 2);
    matrix *p = getSection(T, 0, 2, 3, 3);

    matrix *out = zeros(6,6);

    setSection(out, 0, 2, 0, 2, r);
    setSection(out, 0, 2, 3, 5, matMult( r,hat_R3(p)));

    setSection(out, 3, 5, 3, 5, r);
    setSection(out, 3, 5, 0, 2, zeros(3,3));//todo does this make sense?

    return out;
}

matrix *adj_R6(matrix *z){
    assert(z->numRows == 6);
    assert(z->numCols == 1);

    matrix *r = zeros(6,6);

    //matrix *gu = getSection(r, 0, 2, 0, 0);

    //set top left 3x3
    setSection(r, 0, 2, 0, 2, hat_R3(getSection(z, 3, 5, 0, 0)));

    //set top right 3x3
    setSection(r, 0, 2, 3, 5, hat_R3(getSection(z, 3, 5, 0, 0)));
    return r;
}




//_________________________________________________________________________________________________________

//EXPONENTIAL MAP
//_________________________________________________________________________________________________________

matrix *expm_SO3(matrix *m) {

    double mag = norm(unhat_SO3(m));
    if (mag == 0) {
        return eye(3);
    } else {
        return matrix_add(matrix_add(eye(3),
                                             matrix_scalar_mul(elemDiv(m, mag), sin(mag))),
                                  matrix_scalar_mul(elemDiv(matrixPow(m, 2), pow(mag, 2.0)),
                                                    (1 - cos(mag))));


    }
}



matrix *expm_SE3(matrix *m) {
    assert(m->numRows == 4);
    assert(m->numCols == 4);

    matrix *gW = getSection(m, 0, 2, 0, 2);
    matrix *gU = getSection(m, 0, 2, 3, 3);

    double mag = norm(unhat_SO3(gW));
    matrix *A = eye(3);
    if(mag != 0){

        matrix *k = matrix_scalar_mul(elemDiv(matrixPow(gW, 2), ((double) pow(mag,3.0))) , (mag - (double) sin(mag)));
        matrix *f = matrix_scalar_mul(elemDiv(gW, pow(mag,2.0)) , (1-cos(mag)));
        A = matrix_add(A,matrix_add(k,f));
    }
    matrix *result = zeros(4,4);
    setSection(result, 0, 2, 0, 2, expm_SO3(gW));
    setSection(result, 0, 2, 3, 3, matMult(A, gU));
    setSection(result , 3, 3, 0, 3, zeros(1,4));
    result->data[3][3] = 1;
    return result;
}














