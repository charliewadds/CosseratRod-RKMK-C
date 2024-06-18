//
// Created by Charlie Wadds on 2024-06-18.
//

#include "solvers.h"
#include <nlopt.h>




matrix *fsolve(multisolve_function f, int opts[5]){

    // Dimension of the problem
    int n = 2;

    // Create an optimization object
    nlopt_opt opt = nlopt_create(NLOPT_LD_LBFGS, n); // NLOPT_LD_LBFGS is the algorithm
    //typedef double (*nlopt_func) (unsigned n, const double *x,
    //                              double *gradient, /* NULL if not needed */
    //                              void *func_data);

    // Set the objective function
    nlopt_set_min_objective(opt, f, NULL);

    // Set lower and upper bounds for the variables
    double lb[2] = {-HUGE_VAL, -HUGE_VAL}; // No lower bounds
    double ub[2] = {HUGE_VAL, HUGE_VAL}; // No upper bounds
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);

    // Set the stopping criteria
    nlopt_set_ftol_rel(opt, 1e-6);

    // Initial guess for the variables
    double x[2] = {1.5, 1.5}; // Starting point
    double minf; // Value of the minimum objective function

    // Optimize
    if (nlopt_optimize(opt, x, &minf) < 0) {
        printf("Optimization failed!\n");
    } else {
        printf("Found minimum at f(%g, %g) = %g\n", x[0], x[1], minf);
    }

    // Destroy the optimization object and free memory
    nlopt_destroy(opt);

    return 0;

}