


#ifndef _MATRICES_H_
#define _MATRICES_H_

//#define NEON
#include <stdlib.h>
#include <stdint.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_multiroots.h>





//Define a matrix of doubles
//max number of rows or columns is 255
typedef struct matrix_double_s {
    //there is probably some memory trickery that can be done here to make this more efficient
    int numRows;
    int numCols;
    double *data;
    int square;



} matrix;
double gslDet(gsl_matrix *m);
/*
 * find the exponential mapping of a matrix with pade approximant
 */
matrix *expm(matrix *A, matrix *result);
//return a json string in regular format or matlab format
char* matrixToJson(matrix *m, char *version);

void matrixToFile(matrix *m, char *filename);
matrix *matrixFromFile(char *filename, matrix *result);

gsl_matrix *matrix_to_gsl(matrix *matrix, gsl_matrix *out);
gsl_matrix *copy_matrix_to_gsl(matrix *matrix, gsl_matrix *out);
matrix *gsl_to_matrix(gsl_matrix *gsl_matrix, matrix *result);


int hasNan(matrix *m);
matrix *matrix_solve(matrix *A, matrix *b, matrix *result);
// Constructor-like 
// Allocates memory for a new matrix
// All elements in the matrix are 0.0
matrix *matrix_new(int num_rows, int num_cols);
void copyMatrix(matrix *m, matrix *result);
double matrix_sumSelf(matrix *m);
void printGSLMatrix(gsl_matrix *m);
matrix *matrix_rand(int num_rows, int num_cols);
//return list of the number of rows and columns
int *matrix_shape(matrix *m);

matrix *matMult_elem(matrix *m1, matrix *m2);
matrix *matrix_inverse(matrix *m, matrix *result);
// Dot product
double dot(matrix *m1, matrix *m2);

//todo make most of these use pointers like this: matrix *dot(matrix *result, matrix *m1, matrix *m2);
// Cross product
matrix *cross(matrix *m1, matrix *m2, matrix *result);

// Transpose
matrix *matrix_transpose(matrix *m, matrix *result);

matrix *matrix_sin(matrix *m);

matrix *matrix_outerProduct(matrix *m1, matrix *m2);
// Addition
matrix *matrix_add(matrix *m1, matrix *m2, matrix *result);

matrix *matrix_add3(matrix *m1, matrix *m2, matrix *m3, matrix *result);
// Subtraction
matrix *matrix_sub(matrix *m1, matrix *m2, matrix *result);

// subtraction broadcast
matrix *matrix_sub_broadcast(matrix *m1, matrix *vect);

// Scalar multiplication
matrix *matrix_scalar_mul(matrix *m, double scalar, matrix *result);

// Scalar multiplication chain
//CHAIN FUNCTIONS MUST BE EITHER FREED OR USED IN ANOTHER CHAIN FUNCTION
matrix *matrix_scalar_mul_chain(matrix *m, double scalar);

// convert a matrix to an identity matrix
matrix *eye(matrix *result);

matrix *diag(matrix *A, int n);

//multiply two matrices to create a new matrix
matrix *matMult(matrix *m1, matrix *m2, matrix *result);

//multiply two matrices to create a new matrix
//CHAIN FUNCTIONS MUST BE EITHER FREED OR USED IN ANOTHER CHAIN FUNCTION
matrix *matMult_chain(matrix *m1, matrix *m2);

matrix *matMult_alloc(matrix m1, matrix m2);

matrix *matrixPow(matrix *m, int power, matrix *result);

matrix *matDiv(matrix *m1, matrix *m2);

matrix *elemDiv(matrix *m1, double scalar, matrix *result);

matrix *zeros(int num_rows, int num_cols);

matrix *zeroMatrix(matrix *m);

matrix *ones(int num_rows, int num_cols);

double matrixRatio(matrix *m1, matrix *m2);

void printMatrix(matrix *m);

double norm(matrix *m);

//set a section of a matrix to another matrix. m is the matrix to be added to and section is the matrix to be added
void setSection(matrix *m, int startRow, int endRow, int startCol, int endCol, matrix *section);

matrix *getSection(matrix *m, int startRow, int endRow, int startCol, int endCol, matrix *result);

void getSetSection(matrix *get, matrix *set, int getStartRow, int getEndRow, int getStartCol, int getEndCol, int setStartRow, int setEndRow, int setStartCol, int setEndCol);
//determinant of a matrix
double Det(matrix *m);

//find the domonant eigenvector of a matrix with power iteration todo is this always the dominant eigenvector?
matrix *eigenvector(matrix *m, int iterations);
//find the eigenvalues of the domonant eigenvector of a matrix with the raleigh quotient
double eigenvalue(matrix *m, int iterations);

//characteristic equation of a matrix, i.e. det(A - xI)
double eigenCharEq(matrix *m, double x);
// Destructor-like
void matrix_free(matrix *m);
#endif