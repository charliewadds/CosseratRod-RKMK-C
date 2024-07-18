#include "RobotLib.h"

// Define the function whose roots we want to find
int Flex_MB_BCS_wrapper(const gsl_vector *x, void *params, gsl_vector *f) {
    Flex_MB_BCS_params *p = (Flex_MB_BCS_params *)params;


    // Extracting the elements of x
    double x_arr[6];
    for (int i = 0; i <= x->size-1; i++) {
        x_arr[i] = gsl_vector_get(x, i);
    }

    // Convert x to matrix format
    matrix *x_matrix = zeros(x->size, 1);
    memcpy(x_matrix->data, x_arr, x_matrix->numRows * x_matrix->numCols * sizeof(double));
    // Call Flex_MB_BCS function
    matrix *result;

    //assert(hasNan(x_matrix) == 0);
    result = Flex_MB_BCS(x_matrix, params);
    // Fill f with the residuals
    for (int i = 0; i <= f->size-1; i++) {
        //printf("result: %f\n", result->data[i][0]);
        gsl_vector_set(f, i, result->data[(i * result->numCols)] );
    }
    if(hasNan(x_matrix) == 1){
        return GSL_EINVAL;
    }
    // Free memory
    matrix_free(x_matrix);
    matrix_free(result);

    return GSL_SUCCESS;//todo is this right?
}

void Flex_MB_BCS_wrapper_levmar(double *x, double *f, int m, int n, void *params) {
    Flex_MB_BCS_params *p = (Flex_MB_BCS_params *)params;

    matrix *initGuess = matrix_new(n, 1);
    for (int i = 0; i < n; i++) {
        initGuess->data[i * initGuess->numCols] = x[i];
    }


    matrix *result;
    result = Flex_MB_BCS(initGuess, params);

    for (int i = 0; i < n; i++) {
        f[i] = result->data[i * result->numCols];
    }

}



// Define the function whose roots we want to find
int F_Flex_MB_BCS_wrapper(const gsl_vector *x, void *params, gsl_vector *f) {




    // Extracting the elements of x
    int code = -1;
    double x_arr[x->size];
    for (int i = 0; i < x->size; i++) {
        x_arr[i] = gsl_vector_get(x, i);
    }

    // Convert x to matrix format
    matrix *x_matrix = zeros(x->size, 1);
    for (int i = 0; i < x_matrix->numRows; i++) {
        x_matrix->data[i * x_matrix->numCols] = x_arr[i];
    }

    // Call Flex_MB_BCS function
    matrix *result = zeros(f->size, 1);
    //assert(hasNan(x_matrix) == 0);
    code = F_Flex_MB_BCS(x_matrix, result, params);
    // Fill f with the residuals
    for (int i = 0; i < f->size; i++) {
        //printf("result: %f\n", result->data[i][0]);
        gsl_vector_set(f, i, result->data[(i * result->numCols)] );
    }
    if(hasNan(x_matrix) == 1){
        return GSL_EINVAL;
    }
    // Free memory
    matrix_free(x_matrix);
    matrix_free(result);
    return code;

    return GSL_SUCCESS;//todo is this right?
}

void F_Flex_MB_BCS_wrapper_levmar(double *x, double *f, int m, int n, void *params) {
    Flex_MB_BCS_params *p = (Flex_MB_BCS_params *)params;

    matrix *initGuess = matrix_new(n, 1);
    for (int i = 0; i < n; i++) {
        initGuess->data[i * initGuess->numCols] = x[i];
    }


    matrix *result = matrix_new(m, 1);
    F_Flex_MB_BCS(initGuess, result, params);

    for (int i = 0; i < n; i++) {
        f[i] = result->data[i * result->numCols];
    }

}

int find_roots_levmarqrt(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, double tol) {
//
//    Flex_MB_BCS_params params = {robot, Theta, Theta_dot, Theta_DDot, F_ext, c0, c1, c2};
    //dlevmar_dif(meyer, p, x, m, n, 1000, opts, info, work, covar, NULL); // no



    int n = InitGuess->numRows;
    double *p = malloc(n * sizeof(double));
    for (int i = 0; i < n; i++) {
        p[i] = InitGuess->data[i * InitGuess->numCols];
    }
    double x[n];
    for(int i = 0; i < n; i++){
        x[i] = InitGuess->data[i * InitGuess->numCols];
    }
    double *info = (double *)malloc(10 * sizeof(double));
    double opts[5];
    /* I: opts[0-4] = minim. options [\mu, \epsilon1, \epsilon2, \epsilon3, \delta]. Respectively the
                       * scale factor for initial \mu,
                       * stopping thresholds for ||J^T e||_inf,
                       * ||Dp||_2 and
                       * ||e||_2 and
                       * the step used in difference approximation to the Jacobian. Set to NULL for defaults to be used.
                       * If \delta<0, the Jacobian is approximated with central differences which are more accurate
                       * (but slower!) compared to the forward differences employed by default.
                       */
    /*
     *
     * 0: scale factor for initial mu
     */
    if(fwd) {
        //opts[5] = {1e-3, 1e-9, 1e-5, 1e-5, -1e-9};
        opts[0] = 1e-3; // scale factor for initial mu
        opts[1] = 1e-15; // stopping thresholds for ||J^T e||_inf
        opts[2] = tol; // stopping thresholds for ||Dp||_2
        opts[3] = 1e-18; // stopping thresholds for ||e||_2
        opts[4] = STEP_LEVMAR; // the step used in difference approximation to the Jacobian

    }else{
        //{1e-3, 1e-15, 1e-9, 1e-9, 1e-6};

        opts[0] = 1e-3;
        opts[1] = 1e-15;
        opts[2] = 1e-12;
        opts[3] = tol;
        opts[4] = STEP_LEVMAR;
    }
    assert(isnan(params->robot->objects[1]->object->joint->velocity)==0);
    if(fwd){
        int iters = 0;
        while(iters < MAX_ITER_LEVMAR) {
            dlevmar_dif(F_Flex_MB_BCS_wrapper_levmar, p, NULL, 5, 5, MAX_ITER_LEVMAR - iters, opts, info, NULL, NULL, params);
            iters += info[5];
            if(info[6] == 6){
                break;
            }
            if(info[6] == 2){
                if(info[1] < sqrt(tol)){
                    break;
                }

            }
            if(info[6] == 3){
                break;
            }
        }
    }else {
        int iters = 0;
        int success = 0;
        while(iters < MAX_ITER_LEVMAR) {
            dlevmar_dif(Flex_MB_BCS_wrapper_levmar, p, NULL, 6, 6, MAX_ITER_LEVMAR-iters, opts, info, NULL, NULL, params);
            iters+=info[5]+1;
            if(info[6] == 6){
                success = 1;
                break;
            }
            if(info[6] == 2){
                if(info[1] < sqrt(tol)){
                    success = 1;
                    break;
                }

            }
            if(info[6] == 3){
                break;
            }
        }
        // set iters
        info[5] = iters;
        if(success == 0){
            printf("epsrel ok, epsabs not ok");

            info[6] = 3;
        }

    }

#if SOLVER_SAVE
    if(fwd){
        solverSave("levmar", "levmarSaveFwd.csv", (int)info[6], (int)info[5], (double) info[1], fwd);
    }else {
        solverSave("levmar", "levmarSave.csv", (int)info[6], (int)info[5], (double) info[1], fwd);
    }
#endif

#if VERBOSE >= 2
    printf("iters: %f\n", info[5]);
    printf("reason for terminating: %f\n", info[6]);

    switch ((int)info[6]) {
        case 1:
            printf("stopped by small gradient J^T e\n");
            break;
        case 2:
            printf("stopped by small Dp.\n");
            break;
        case 3:
            printf("max number of iterations reached\n");
            break;
        case 4:
            printf("singular matrix. Restart from current p with increased mu\n");
            break;
        case 5:
            printf("no further error reduction is possible. Restart with increased mu\n");
            break;
        case 6:
            printf("stopped by small ||e||_2 i.e. solver converged\n");
            break;
        case 7:
            printf("stopped by invalid (i.e. NaN or Inf) function values. Restart with current p\n");
            break;
        case 8:
            printf("stopped by parameter limits\n");
            break;
        default:
            printf("unknown reason for termination\n");
            break;

    }


    printf("\te_2 @initial p %.30f\n", info[0]);
    printf("\te_2 %.30f\n", info[1]);
    printf("\tJ^T e %.30f\n", info[2]);
    printf("\t||Dp||_2 %.30f\n", info[3]);
    printf("\tmu/max[J^T J] %.30f\n", info[4]);
    printf("\titers: %f\n", info[5]);
    printf("\tevals: %f\n", info[7]);
    printf("\tjacobian evals: %f\n", info[8]);
    printf("\tlinear system solves: %f\n", info[9]);

    printf("\tSolution\n");
    printMatrix(InitGuess);
    printf("the residual is: %.30f\n", info[1]);
#endif
    for (int i = 0; i < InitGuess->numRows; i++) {
        InitGuess->data[i * InitGuess->numCols] = p[i];
    }
    return info[6];
}


int find_roots_newton(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, double tol) {
    const gsl_multiroot_fsolver_type *T;
    //gsl_multiroot_fsolver *s;
    assert(hasNan(InitGuess) == 0);
    T = gsl_multiroot_fsolver_dnewton;
    //s = gsl_multiroot_fsolver_allc(T, 6);
    int status;
    size_t iter = 0;

    const size_t n = InitGuess->numRows; // Number of variables



    gsl_multiroot_function f = {&Flex_MB_BCS_wrapper, n, params};
    // Set parameters
    if(fwd) {

        f.f = &F_Flex_MB_BCS_wrapper;
    }

    double x_init[InitGuess->numRows];

    for (int i = 0; i < InitGuess->numRows; i++) {
        x_init[i] = InitGuess->data[i * InitGuess->numCols];
    }

    gsl_vector_view x_vec = gsl_vector_view_array(x_init, n);
    gsl_multiroot_fsolver *s = gsl_multiroot_fsolver_alloc(T, n);
    gsl_multiroot_fsolver_set(s, &f, &x_vec.vector);


    do {
        iter++;
        status = gsl_multiroot_fsolver_iterate(s);

#if VERBOSE >= 2
        if (status) {
            printf("STATUS: %s\n", gsl_strerror(status));
            break;
        }
#endif


        status = gsl_multiroot_test_residual(s->f, tol);

        if(status == GSL_CONTINUE){
            if(fwd) {
                status = gsl_multiroot_test_delta(s->dx, s->x, tol, pow(tol,2));
            }else {
                status = gsl_multiroot_test_delta(s->dx, s->x, tol, pow(tol,2));
            }
        }

    } while (status == GSL_CONTINUE && iter < MAX_ITER_HYBRID);


#if SOLVER_SAVE ==1
    double residual = 0;
    residual = gsl_blas_dnrm2(s->f);
    if(fwd){
        solverSave("hybrid", "hybridSaveFwd.csv", status, (int)iter, residual, 1);
    }else {
        solverSave("hybrid", "hybridSave.csv", status, (int) iter, residual, fwd);
    }
#endif
#if VERBOSE >= 2
    if(fwd) {
        if (status != GSL_SUCCESS) {
            printf("\tSTATUS: %d\n", status);
            printf("\tSTATUS: %s\n", gsl_strerror(status));
        }
        printf("\tnewton took %zu iterations\n", iter);\

    }
    else{
        if (status != GSL_SUCCESS){
            printf("STATUS: %d\n", status);
            printf("STATUS: %s\n", gsl_strerror(status));
        }
        printf("newton took %zu iterations\n", iter);
    }
//    gsl_matrix *jacobian = gsl_matrix_alloc(n, n);
//    int stat = gsl_multiroot_fdjacobian(&f, &x_vec.vector, s->f, 1e-4, jacobian);
//    printf("JACOBIAN:\n");
//    printf("STATUS: %d\n", stat);
//    printGSLMatrix(jacobian);
//
//    matrix *determinant = zeros(6,6);
//    gsl_to_matrix(jacobian, determinant);
//    printf("DETERMINANT: %.15f\n", Det(determinant));
//    assert(fabs(Det(determinant)) > 0.0000000000000000000000000001);
#endif



    //assert(!isnan(s->f->data[1]));
    // Extract solution
    //matrix *solution = zeros(6, 1);
    for (int i = 0; i < InitGuess->numRows; i++) {
        InitGuess->data[i * InitGuess->numCols] = gsl_vector_get(s->x, i);
    }

    gsl_multiroot_fsolver_free(s);
    return status;
}

int find_roots_hybrid(matrix *InitGuess, Flex_MB_BCS_params *params, int fwd, double tol) {
    const gsl_multiroot_fsolver_type *T;
    //gsl_multiroot_fsolver *s;
    assert(hasNan(InitGuess) == 0);
    T = gsl_multiroot_fsolver_hybrids;
    //s = gsl_multiroot_fsolver_allc(T, 6);
    int status;
    size_t iter = 0;

    const size_t n = InitGuess->numRows; // Number of variables



    gsl_multiroot_function f = {&Flex_MB_BCS_wrapper, n, params};
    // Set parameters
    if(fwd) {

        f.f = &F_Flex_MB_BCS_wrapper;
    }

    double x_init[InitGuess->numRows];

    for (int i = 0; i < InitGuess->numRows; i++) {
        x_init[i] = InitGuess->data[i * InitGuess->numCols];
    }

    gsl_vector_view x_vec = gsl_vector_view_array(x_init, n);
    gsl_multiroot_fsolver *s = gsl_multiroot_fsolver_alloc(T, n);
    gsl_multiroot_fsolver_set(s, &f, &x_vec.vector);


    do {
        iter++;
        status = gsl_multiroot_fsolver_iterate(s);

#if VERBOSE >= 2
        if (status) {
            printf("STATUS: %s\n", gsl_strerror(status));
            break;
        }
#endif

        status = gsl_multiroot_test_residual(s->f, tol);

        if(status == GSL_CONTINUE){
            //status = gsl_multiroot_test_delta(s->dx, s->x, EPSABS_HYBRID, EPSREL_HYBRID);
            double result;
            gsl_blas_ddot(s->f, s->f, &result);
            if(gsl_blas_dnrm2(s->dx) < EPSREL_HYBRID && result < sqrt(tol)){
                status = GSL_SUCCESS;
            }
        }

//        if(status == GSL_CONTINUE){
//            if(fwd) {
//                status = gsl_multiroot_test_delta(s->dx, s->x, tol, tol);
//
//            }else {
//                status = gsl_multiroot_test_delta(s->dx, s->x, tol, tol);
//            }
//        }

    } while (status == GSL_CONTINUE && iter < MAX_ITER_HYBRID);


#if SOLVER_SAVE ==1
    double residual = 0;
    residual = gsl_blas_dnrm2(s->f);
    if(fwd){
        solverSave("hybrid", "hybridSaveFwd.csv", status, (int)iter, residual, 1);
    }else {
        solverSave("hybrid", "hybridSave.csv", status, (int) iter, residual, fwd);
    }
#endif
#if VERBOSE >= 2
    if(fwd) {
        if (status != GSL_SUCCESS) {
            printf("\tSTATUS: %d\n", status);
            printf("\tSTATUS: %s\n", gsl_strerror(status));
        }
        printf("\thybrid took %zu iterations\n", iter);
        printf("\tthe residual is: %.30f\n", gsl_blas_dnrm2(s->f));

    }
    else{
            if (status != GSL_SUCCESS){
                printf("STATUS: %d\n", status);
                printf("STATUS: %s\n", gsl_strerror(status));

            }
        printf("\tthe residual is: %.30f\n", gsl_blas_dnrm2(s->f));
            printf("hybrid took %zu iterations\n", iter);
    }
//    gsl_matrix *jacobian = gsl_matrix_alloc(n, n);
//    int stat = gsl_multiroot_fdjacobian(&f, &x_vec.vector, s->f, 1e-4, jacobian);
//    printf("JACOBIAN:\n");
//    printf("STATUS: %d\n", stat);
//    printGSLMatrix(jacobian);
//
//    matrix *determinant = zeros(6,6);
//    gsl_to_matrix(jacobian, determinant);
//    printf("DETERMINANT: %.15f\n", Det(determinant));
//    assert(fabs(Det(determinant)) > 0.0000000000000000000000000001);
#endif



    //assert(!isnan(s->f->data[1]));
    // Extract solution
    //matrix *solution = zeros(6, 1);
    for (int i = 0; i < InitGuess->numRows; i++) {
        InitGuess->data[i * InitGuess->numCols] = gsl_vector_get(s->x, i);
    }

    gsl_multiroot_fsolver_free(s);
    return status;
}