#ifndef KALMAN_H
#define KALMAN_H
#define L 5
#include <array>
#include <vector>
#include "init.h"


class KalmanFilter {
public:
    // Initialization of the filter
    void init(const std::array<double, N>& x0,
              const std::array<std::array<double, N>, N>& P0,
              const std::array<std::array<double, N>, N>& Q_,
              double R_,
              const std::array<std::array<double, N>, N>& G_,
              const std::array<double, N>& H_,
              const std::array<double, N>& C_);

    // UKF prediction step
    void predict(double u, const std::array<double, N>& process_noise);

    // UKF correction step
    void correction(double y, double measurement_noise);

    // Generate sigma points based on current state and covariance
    std::array<std::array<double, N>, L> generateSigmaPoints();

    // Compute the weighted mean of propagated sigma points
    std::array<double, N> computeWeightedMean(const std::vector<std::array<double, N>>& sigma_points, 
                                              const std::array<double, L>& weights);

    // Compute the weighted covariance of propagated sigma points
    std::array<std::array<double, N>, N> computeWeightedCovariance(const std::vector<std::array<double, N>>& sigma_points, 
                                                                    const std::array<double, N>& mean, 
                                                                    const std::array<double, L>& weights);

private:
    std::array<double, N> x; // State vector
    std::array<std::array<double, N>, N> P; // Covariance matrix
    std::array<std::array<double, N>, N> Q; // Process noise covariance
    double R; // Measurement noise covariance
    std::array<std::array<double, N>, N> G; // Process matrix
    std::array<double, N> H; // Measurement matrix
    std::array<double, N> C; // Control matrix
};

#endif // KALMAN_H

