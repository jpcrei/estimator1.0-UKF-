#include "init.h"

double sigma[N][N] = {{0.1, 0.0}, {0.0, 0.1}}; // Initial covariance matrix
const double mu[N] = {1.0, 2.0}; // Mean vector

// Parameters for calculating sigma
const double alpha = 0.9; 
const double beta = 2.0; 
const double k = 1.0; 
double lambda;

void initialize_parameters() {
    lambda = ((alpha * alpha) * (N + k) - N);
}
