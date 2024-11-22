#ifndef INIT_H
#define INIT_H

#include <array>
#define N 2


// Initial covariance matrix, mean vector, and other parameters
extern double sigma[N][N];
extern const double mu[N];

// Function to initialize parameters
void initialize_parameters();

#endif // INIT_H
