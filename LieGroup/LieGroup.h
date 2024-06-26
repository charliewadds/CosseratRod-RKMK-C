/*
 * Created by Charlie Wadds on 2024-02-28.
 *
*/

#ifndef COSSERATROD_RKMK_C_LIEGROUP_H
#define COSSERATROD_RKMK_C_LIEGROUP_H
#include "Matrices.h"
#define PI M_PI
//todo could define R3 column vector if it is used a lot

//todo could get rid of SO3 and SE3 and just use matrix if that is faster, this is more readable, shouldnt significanty affect speed just marginally more memory
//alternatively could re-implement matrices to make calling it cleaner
typedef struct SO3_s {
    matrix *R;//3x3 rotation matrix
}SO3;
//SE3 functions
SO3 *new_SO3(matrix *R);
void free_SO3(SO3 *T);
SO3 *new_SO3_zeros();

//todo add se2

// SE3 4x4 matrix todo I dont think this is a good way to do this
typedef struct SE3_s {
    SO3 *R;//so3 rotation matrix
    matrix *P;//position 3x1 matrix
    matrix *T;//4x4 matrix with R and p and [0,0,0,1] in the bottom row

}SE3;

//SE3 functions
SE3 *new_SE3(matrix *R, matrix *P);
void free_SE3(SE3 *T);
SE3 *new_SE3_zeros();
SE3 *new_SE3_T(matrix *T);



// 3x1 column vector -> hat operator -> 3x3 matrix in so(3)
matrix *hat_R3(matrix *z, matrix *result);

// 6x1 column vector e.g. [p;p;p;r;r;r] -> hat operator -> 4x4 matrix in se(3)
matrix *hat_R6(matrix *z, matrix *result);

// 6x1 column vector e.g. [p;p;p;r;r;r] -> hat operator -> 4x4 matrix in se(3)
//free z after use, CHAIN FUNCTIONS MUST BE EITHER FREED OR USED IN ANOTHER CHAIN FUNCTION
matrix *hat_R6_chain(matrix *z);
// 3x3 matrix in so(3) -> unhat operator -> 3x1 column vector
matrix *unhat_SO3(matrix *zhat, matrix *result);//return a 3x1 vector from a 3x3 SO3 matrix
// 4x4 matrix in se(3) -> unhat operator -> 6x1 column vector
matrix *unhat_SE3(matrix *zhat, matrix *result);//return a 6x1 vector from a 4x4 SE3 matrix

// 4x4 matrix in SE3 -> adj operator -> 6x6 matrix in ????
matrix *adj(matrix *T, matrix *result);

// 4x4 matrix in SE3 -> adj operator -> 6x6 matrix in ????
//free T after use, CHAIN FUNCTIONS MUST BE EITHER FREED OR USED IN ANOTHER CHAIN FUNCTION
matrix *adj_chain(matrix *T);

// 6x1 column vector -> adjoint operator -> 6x6 matrix in ????
matrix *adj_R6(matrix *z, matrix *result);

// mapping from so(3) to SO(3)
// 3x3 matrix in so(3) -> expm operator -> 3x3 matrix in SO(3)
matrix *expm_SO3(matrix *w, matrix *result);

//mapping from se(3) to SE(3)
// 4x4 matrix in se(3) -> expm operator -> 4x4 matrix in SE(3)
matrix *expm_SE3(matrix *z, matrix *result);

//mapping from se(3) to SE(3)
// 4x4 matrix in se(3) -> expm operator -> 4x4 matrix in SE(3)
//free z after use, CHAIN FUNCTIONS MUST BE EITHER FREED OR USED IN ANOTHER CHAIN FUNCTION
matrix *expm_SE3_chain(matrix *m);

#endif //COSSERATROD_RKMK_C_LIEGROUP_H
