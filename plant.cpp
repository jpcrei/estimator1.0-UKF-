#include "plant.h"
#include <cmath>
#include <array>
#include "init.h"


// Process model: Propagate one sigma point through the state equation
std::array<double, N> plant_process(const std::array<double, N>& x, double u, const std::array<double, N>& process_noise, double Rw) {
    std::array<double, N> x_new; // Updated state
    std::array<double, N> w_k;

    // Process noise
    for (size_t i = 0; i < N; ++i) {
        w_k[i] = std::sqrt(Rw) * process_noise[i];
    }

    // State update equations (example equations)
    x_new[0] = x[1] * u + w_k[0];        // Example: linear combination of input
    x_new[1] = -0.5 * x[0] + u + w_k[1]; // Example: some nonlinear dynamics

    return x_new;
}

// Measurement model: Compute measurement for one sigma point
double plant_measurement(const std::array<double, N>& x, double u, const std::array<double, 2>& measurement_noise, double Rv) {
    double measurement = x[0]; // Example: C * x
    return measurement + std::sqrt(Rv) * measurement_noise[0];
}
