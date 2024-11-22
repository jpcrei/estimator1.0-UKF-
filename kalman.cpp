#include "kalman.h"
#include "plant.h" // For plant_process and plant_measurement
#include <cmath>
#include <vector>
#include "init.h"



void KalmanFilter::init(const std::array<double, N>& x0,
                        const std::array<std::array<double, N>, N>& P0,
                        const std::array<std::array<double, N>, N>& Q_,
                        double R_,
                        const std::array<std::array<double, N>, N>& G_,
                        const std::array<double, N>& H_,
                        const std::array<double, N>& C_) {
    x = x0;
    P = P0;
    Q = Q_;
    R = R_;
    G = G_;
    H = H_;
    C = C_;
}

// Generate sigma points
std::array<std::array<double, N>, L> KalmanFilter::generateSigmaPoints() {
    std::array<std::array<double, N>, L> sigma_points;
    double lambda = 3 - N; // Scaling parameter for sigma points
    std::array<std::array<double, N>, N> sqrtP; // Cholesky decomposition of P

    // Populate sigma points
    sigma_points[0] = x; // Central sigma point
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            sigma_points[i + 1][j] = x[j] + std::sqrt(lambda + N) * sqrtP[i][j];
            sigma_points[i + 1 + N][j] = x[j] - std::sqrt(lambda + N) * sqrtP[i][j];
        }
    }

    return sigma_points;
}

// Predict step
void KalmanFilter::predict(double u, const std::array<double, N>& process_noise) {
    auto sigma_points = generateSigmaPoints();
    std::vector<std::array<double, N>> propagated_sigma_points(L);

    // Propagate each sigma point through the plant model
    for (size_t i = 0; i < L; ++i) {
        propagated_sigma_points[i] = plant_process(sigma_points[i], u, process_noise, Q[0][0]);
    }

    // Compute predicted mean and covariance
    std::array<double, N> weights;
    weights.fill(1.0 / L); // Uniform weights
    x = computeWeightedMean(propagated_sigma_points, weights);
    P = computeWeightedCovariance(propagated_sigma_points, x, weights);
}

// Correction step
void KalmanFilter::correction(double y, double measurement_noise) {
    auto sigma_points = generateSigmaPoints();
    std::vector<double> predicted_measurements(L);

    // Transform sigma points through the measurement model
    for (size_t i = 0; i < L; ++i) {
        predicted_measurements[i] = plant_measurement(sigma_points[i], 0.0, {0.0, 0.0}, R);
    }

    // Compute predicted measurement mean and covariance
    double y_mean = 0.0;
    for (double m : predicted_measurements) y_mean += m / L;

    double S = 0.0; // Innovation covariance
    for (double m : predicted_measurements) S += std::pow(m - y_mean, 2) / L;
    S += R;

    // Compute Kalman gain
    std::array<double, N> K;
    for (int i = 0; i < N; ++i) {
        K[i] = P[i][0] / S; // Example computation
    }

    // Update state and covariance
    double innovation = y - y_mean;
    for (size_t i = 0; i < N; ++i) {
        x[i] += K[i] * innovation;
    }
}

