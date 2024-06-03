//
// Created by Charlie Wadds on 2024-02-28.
//

#include <math.h>
#include <assert.h>
#include <string.h>
#include "LieGroup.h"





//_________________________________________________________________________________________________________


//HAT OPERATORS
//_________________________________________________________________________________________________________

matrix *hat_R3(matrix *z, matrix *result){

    assert(z->numRows == 3);
    assert(z->numCols == 1);

    assert(result->numRows == 3);
    assert(result->numCols == 3);
    //matrix *T = zeros(3,3);



    result->data[(0 * result->numCols) + 1] = z->data[(2 * z->numCols)] * -1;
    result->data[(0 * result->numCols) + 2] = z->data[(1 * z->numCols)];

    result->data[(1 *result->numCols)+0] = z->data[(2 * z->numCols)];
    result->data[(1 *result->numCols)+2] = z->data[(0 * z->numCols)] * -1;

    result->data[(2 *result->numCols)+0] = z->data[(1 * z->numCols)] * -1;
    result->data[(2 *result->numCols)+1] = z->data[(0 * z->numCols)];


    return result;

}

matrix *hat_R6(matrix *z, matrix *result){
    assert(z->numRows == 6);
    assert(z->numCols == 1);

    assert(result->numRows == 4);
    assert(result->numCols == 4);


    matrix *temp = matrix_new(3,3);
    matrix *temp3x1 = matrix_new(3,1);

    matrix *tempResult;
    if(z == result){
        tempResult = matrix_new(4,4);
    }else{
        tempResult = result;
        zeroMatrix(tempResult);
    }

    setSection(tempResult, 0, 2, 0, 2, hat_R3(getSection(z, 3, 5, 0, 0, temp3x1), temp));
    setSection(tempResult, 0, 2, 3, 3, getSection(z, 0, 2, 0, 0, temp3x1));
    //getSetSection(z, result,0,2,0,0,0,2,3,5);
    //setSection(result, 3, 3, 0, 3, zeros(1,4));//todo convert to zeroSection using memset
    //T->data[3][3] = 1;//todo this is not in matlab, dont I need it?

    matrix_free(temp);
    matrix_free(temp3x1);
    copyMatrix(tempResult, result);
    //matrix_free(tempResult);

    if(z == result){
        copyMatrix(tempResult, result);
        matrix_free(tempResult);
    }

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
    matrix *temp;
    if(zhat == result) {
        temp = matrix_new(3, 1);
    }else{
        temp = result;
    }
    temp->data[(0 * temp->numCols) + 0] = zhat->data[(2 * zhat->numCols) + 1];
    temp->data[(1 * temp->numCols) + 0] = zhat->data[(0 * zhat->numCols) + 2];
    temp->data[(2 * temp->numCols) + 0] = zhat->data[(1 * zhat->numCols) + 0];

    if(zhat == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }

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


//    [0.07247618 0.12745597 0.64657566 0.70431103]
//    [0.36749106 0.67756713 0.97582436 0.55378821]
//    [0.20491832 0.86476807 0.42959704 0.97524496]
//    [0.69906914 0.44438659 0.85453082 0.67142751]

    matrix *tempRes;

    if(T == result){
        tempRes = matrix_new(6,6);
    }else{
        tempRes = result;
        zeroMatrix(tempRes);
    }
    // Create submatrices
    matrix *r = matrix_new(3, 3);
    getSection(T, 0, 2, 0, 2, r);

    matrix *p = matrix_new(3, 1);
    getSection(T, 0, 2, 3, 3, p);

    matrix *temp = matrix_new(3, 3);

    // Set the first 3x3 block of the result matrix to r
    setSection(tempRes, 0, 2, 0, 2, r);

    // Compute hat_R3(p) * r and set it in the result matrix
    hat_R3(p, temp);
    matMult(temp, r, temp);
    setSection(tempRes, 0, 2, 3, 5, temp);

    // Set the bottom-right 3x3 block of the result matrix to r
    setSection(tempRes, 3, 5, 3, 5, r);

    // Free temporary matrices
    matrix_free(p);
    matrix_free(r);
    matrix_free(temp);

    if(T == result){
        copyMatrix(tempRes, result);
        matrix_free(tempRes);
    }
    //matrix_free(tempIn);

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
    matrix *tempOut;
    if(z == result) {
        tempOut = matrix_new(6, 6);
    }else{
        tempOut = result;
        zeroMatrix(tempOut);
    }

    matrix *gu = matrix_new(3,1);
    getSection(z, 0, 2, 0, 0, gu);

    matrix *gw = matrix_new(3,1);
    getSection(z, 3, 5, 0, 0, gw);

    //matrix *r = zeros(6,6);
    matrix *temp = matrix_new(3,3);
    setSection(tempOut, 0, 2, 0, 2, hat_R3(gw, temp));
    setSection(tempOut, 0, 2, 3, 5, hat_R3(gu, temp));
    setSection(tempOut, 3, 5, 3, 5, hat_R3(gw, temp));

    matrix_free(gw);
    matrix_free(gu);
    matrix_free(temp);

    if(z == result){
        copyMatrix(tempOut, result);
        matrix_free(tempOut);
    }



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
    matrix *tempOut;

    if(m == result) {
        tempOut = matrix_new(3, 3);
    }else{
        tempOut = result;
        //zeroMatrix(tempOut);
    }
    matrix *temp = matrix_new(3,1);
    double mag = norm(unhat_SO3(m, temp));

    matrix *temp_3x3n1 = matrix_new(3,3);
    matrix *temp_3x3n2 = matrix_new(3,3);


    if (mag == 0) {
        eye(tempOut);
    } else {
        matrix_add(
                matrix_add(
                        eye(tempOut),matrix_scalar_mul(elemDiv(m, mag, temp_3x3n1), sin(mag), temp_3x3n1)
                              , temp_3x3n1),
                                  matrix_scalar_mul(
                                          elemDiv(
                                                  matrixPow(m, 2, temp_3x3n2), pow(mag, 2.0),
                                                  temp_3x3n2
                                                  ),
                                                  (1 - cos(mag))
                                          , temp_3x3n2)
                        , tempOut);


    }

    matrix_free(temp);
    matrix_free(temp_3x3n1);
    matrix_free(temp_3x3n2);

    if(m == result){
        copyMatrix(tempOut, result);
        matrix_free(tempOut);
    }

    return result;
}



matrix *expm_SE3(matrix *G, matrix *result) {
    assert(G->numRows == 4 && G->numCols == 4);
    assert(result->numRows == 4 && result->numCols == 4);

    matrix *temp ;
    if(G == result){
        temp = matrix_new(4, 4);
    }else{
        temp = result;

    }

    //G = tempG;

    //zeroMatrix(result);

    matrix *Gw = matrix_new(3, 3);
    getSection(G, 0, 2, 0, 2, Gw);

    matrix *Gu = matrix_new(3, 1);
    getSection(G, 0, 2, 3, 3, Gu);

    matrix *temp3x1 = matrix_new(3, 1);
    matrix *temp3x3 = matrix_new(3, 3);
    double magGw = norm(unhat_SO3(Gw, temp3x1));

    matrix *A = eye(matrix_new(3, 3));

    if (magGw != 0) {
        matrix *Gw2 = matrix_new(3, 3);
        matrixPow(Gw, 2, Gw2);
        matrix *term1 = matrix_new(3, 3);
        matrix *term2 = matrix_new(3, 3);

        matrix_scalar_mul(elemDiv(Gw2, pow(magGw, 3), term1), magGw - sin(magGw), term1);
        matrix_scalar_mul(elemDiv(Gw, pow(magGw, 2), term2), 1 - cos(magGw), term2);

        matrix_add3(A, term1, term2, A);

        matrix_free(Gw2);
        matrix_free(term1);
        matrix_free(term2);
    }

    expm_SO3(Gw, temp3x3);
    setSection(temp, 0, 2, 0, 2, temp3x3);

    matMult(A, Gu, temp3x1);
    setSection(temp, 0, 2, 3, 3, temp3x1);

    temp->data[(3 * temp->numCols) + 3] = 1;

    matrix_free(Gw);
    matrix_free(Gu);
    matrix_free(temp3x1);
    matrix_free(temp3x3);
    matrix_free(A);
    if(G == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }


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
    m->data[(3 * m->numCols) + 3] = 1;


    matrix_free(gW);
    matrix_free(gU);
    matrix_free(temp3x3);
    matrix_free(A);

    return m;
}














