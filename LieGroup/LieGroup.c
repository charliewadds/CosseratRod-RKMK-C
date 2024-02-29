//
// Created by Charlie Wadds on 2024-02-28.
//

#include "LieGroup.h"



//CONSTRUCTORS
//______________________________________________________________________
SO3 *new_SO3(matrix *R){
    SO3 *T = (SO3 *)malloc(sizeof(SO3));
    T->R = R;
    return T;
}


SO3 *new_SO3_zeros(){
    matrix *R = matrix_new(3,3);
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
    T->R->R = R;
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
matrix *T_from_PR(matrix *R, matrix *P){
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
    T->R->data[0][1] = -z->data[2][0];
    T->R->data[0][2] = z->data[1][0];

    T->R->data[1][0] = z->data[2][0];
    T->R->data[1][2] = -z->data[0][0];

    T->R->data[2][0] = -z->data[1][0];
    T->R->data[2][1] = z->data[0][0];
    return T;
}

SE3 *hat_R6(matrix *z){
    SE3 *T_hat = new_SE3_zeros();
    matrix *z_w = matrix_new(3,1);
    matrix *z_v = matrix_new(3,1);

    //todo there is probably a more elegant way to do this
    z_w->data[0] = z->data[3];
    z_w->data[1] = z->data[4];
    z_w->data[2] = z->data[5];

    z_v->data[0] = z->data[0];
    z_v->data[1] = z->data[1];
    z_v->data[2] = z->data[2];

    T_hat->R = hat_R3(z_w);
    T_hat->P = z_v;//position vector is the same
    T_hat->T = T_from_PR(T_hat->R->R, T_hat->P);
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
    setSection(r, 0, 2, 3, 5, matMult(matrix_new(3,3), T->R->R));

    //set bottom left 3x3, zeros
    setSection(r, 3, 5, 0, 2, zeros(3,3));

    //set bottom right 3x3
    setSection(r, 3, 5, 3, 5, T->R->R);

    return r;
}

matrix *adj_R6(matrix *z){
    matrix *r = matrix_new(6,6);

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
//
//SO3 *exp_SO3(SO3 *m){
//    SO3 *z = new_SO3_zeros();
//
//
//
//}
















