//
// Created by Charlie Wadds on 2024-07-04.
//
#include "SolversV1.h"
#include <gsl/gsl_blas.h>




int find_roots_newton_V1(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, BCS_func bcs_f) {
    const gsl_multiroot_fsolver_type *T;
    gsl_multiroot_fsolver *s;

    T = gsl_multiroot_fsolver_dnewton;
    //s = gsl_multiroot_fsolver_allc(T, 6);
    int status;
    size_t iter = 0;

    const size_t n = InitGuess->numRows; // Number of variables

    gsl_multiroot_function *f = malloc(sizeof(gsl_multiroot_function));
    // Set parameters
    if(fwd == 1){
        f->f = &F_Flex_MB_BCS_wrapper;
        f->n = n;
        f->params = params;

    }else {
        f->f = ;
        f->n = n;
        f->params = params;
    }
    //f.params = &params;

    // Define initial guess
    double x_init[n];

    for (int i = 0; i < n; ++i) {
        x_init[i] = InitGuess->data[i * InitGuess->numCols];
    }

    gsl_vector_view x_vec = gsl_vector_view_array(x_init, n);
    s = gsl_multiroot_fsolver_alloc(T, n);
    gsl_multiroot_fsolver_set(s, f, &x_vec.vector);


#if VERBOSE == 1
    // output the jacobian
    gsl_matrix *jacobian = gsl_matrix_alloc(n, n);
    int stat = gsl_multiroot_fdjacobian(f, &x_vec.vector, s->f, 1e-9, jacobian);
    printf("JACOBIAN:\n");
    printf("STATUS: %d\n", stat);
    printGSLMatrix(jacobian);
#endif
    do {
        iter++;
        status = gsl_multiroot_fsolver_iterate(s);
#if VERBOSE == 1
        if (status) {
            printf("STATUS: %s\n", gsl_strerror(status));
            break;
        }
#endif

        if(fwd){
            status = gsl_multiroot_test_residual(s->f, TOLERANCE_FWD);
        }else{
            status = gsl_multiroot_test_residual(s->f, TOLERANCE_INV);

        }

    } while (status == GSL_CONTINUE && iter < MAX_ITER_NEWTON);


    for (int i = 0; i < InitGuess->numRows; ++i) {
        InitGuess->data[i * InitGuess->numCols] = gsl_vector_get(s->x, i);
    }

    gsl_multiroot_fsolver_free(s);
    free(f);

    return status;
}


int fSolver(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, BCS_func f, int solver){

    if(solver == 0){
        return find_roots_newton_V1(InitGuess, params, fwd, f);
    }else if(solver == 1){
        return find_roots_hybrid_V1(InitGuess, params, fwd, f);
    }else if(solver == 2){
        return find_roots_levmarqrt_V1(InitGuess, params, fwd, f);
    }else{
        return -1;
    }
}