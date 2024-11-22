#include <Arduino.h>
#include "noise.h"
#include "kalman.h"
#include "plant.h"
#include "init.h"


std::array<double, N> x0 = {0.0, 0.0};
std::array<std::array<double, N>, N> P0 = {{{1.0, 0.0}, {0.0, 1.0}}};
std::array<std::array<double, N>, N> Q = {{{0.001, 0.0}, {0.0, 0.001}}}; // Process noise covariance (Rw)
double R = 1.2; // Measurement noise covariance (Rv)
double dt = 0.1; // Time step
int steps = 100;

// UKF weights
std::array<double, L> weights;

// State and sigma points
std::array<double, N> x;
std::array<std::array<double, N>, L> sigma_points;
std::vector<std::array<double, N>> propagated_sigma_points(L);

// Kalman filter
KalmanFilter kf;

void setup() {
    Serial.begin(9600);

    initialize_parameters(); // Initialize global parameters
    kf.init(x0, P0, Q, R, {}, {}, {}); // UKF doesn't require matrices G, H, and C explicitly

    // Set initial UKF weights
    double lambda = 3 - N; // Scaling parameter
    weights[0] = lambda / (lambda + N);
    for (size_t i = 1; i < L; ++i) {
        weights[i] = 1.0 / (2.0 * (lambda + N));
    }
}

void loop() {
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        double u = sin(2 * PI * t); // Sine wave input

        // Generate sigma points
        sigma_points = kf.generateSigmaPoints();

        // Propagate sigma points through the process model
        for (size_t j = 0; j < L; ++j) {
            std::array<double, N> process_noise = {gaussianNoise(), gaussianNoise()};
            propagated_sigma_points[j] = plant_process(sigma_points[j], u, process_noise, Q[0][0]);
        }

        // Compute predicted mean and covariance
        x = kf.computeWeightedMean(propagated_sigma_points, weights);
        auto P = kf.computeWeightedCovariance(propagated_sigma_points, x, weights);

        // Transform sigma points through the measurement model
        std::vector<double> predicted_measurements(L);
        for (size_t j = 0; j < L; ++j) {
            std::array<double, 2> measurement_noise = {gaussianNoise(), 0.0};
            predicted_measurements[j] = plant_measurement(propagated_sigma_points[j], u, measurement_noise, R);
        }

        // Compute predicted measurement mean and covariance
        double y_mean = 0.0;
        for (size_t j = 0; j < L; ++j) {
            y_mean += weights[j] * predicted_measurements[j];
        }

        double S = 0.0; // Measurement covariance
        for (size_t j = 0; j < L; ++j) {
            S += weights[j] * (predicted_measurements[j] - y_mean) * (predicted_measurements[j] - y_mean);
        }
        S += R;

        // Compute Kalman gain
        std::array<double, N> K;
        for (size_t j = 0; j < N; ++j) {
            K[j] = P[j][0] / S;
        }

        // Update state and covariance
        double y_actual = plant_measurement(x, u, {0.0, 0.0}, R); // Actual measurement
        double innovation = y_actual - y_mean;
        for (size_t j = 0; j < N; ++j) {
            x[j] += K[j] * innovation;
        }

        // Update covariance matrix
        for (size_t j = 0; j < N; ++j) {
            for (size_t k = 0; k < N; ++k) {
                P[j][k] -= K[j] * S * K[k];
            }
        }

        // Print results
        Serial.print(y_actual);
        Serial.print(",");
        Serial.println(x[0]);
    }

    while (true); // Halt execution after simulation
}













