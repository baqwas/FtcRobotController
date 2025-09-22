/*
 * MIT License
 *
 * Copyright (c) 2024 ParkCircus Productions; All Rights Reserved
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Holonomic;
// This file contains the core implementation of the Kalman Filter.
// It is designed to be used by the MecanumSensorFusion OpMode.

// Note: This implementation uses a simplified Matrix library for clarity.
// In a real-world application, you might use a more robust library like EJML.
// The key is to understand the matrix operations.

public class KalmanFilter {

    // The state vector: [x, y, heading, dx, dy, d_heading]
    private double[] x_hat; // Estimated state
    private double[][] P;   // Covariance matrix

    // Pre-defined matrices for the model
    private final double[][] F; // State transition matrix
    private final double[][] B; // Control input matrix
    private final double[][] Q; // Process noise covariance
    private final double[][] H; // Observation matrix
    private final double[][] R; // Measurement noise covariance

    public KalmanFilter(double[] initial_x, double[][] initial_P, double[][] F, double[][] B, double[][] Q, double[][] H, double[][] R) {
        // Initialize state and covariance
        this.x_hat = initial_x;
        this.P = initial_P;

        // Initialize model matrices
        this.F = F;
        this.B = B;
        this.Q = Q;
        this.H = H;
        this.R = R;
    }

    /**
     * Prediction step: Project the state and covariance forward in time.
     * @param u The control vector, e.g., motor powers.
     */
    public void predict(double[] u) {
        // Predict new state: x_hat_k = F * x_hat_k-1 + B * u
        double[] F_x = Matrix.multiply(F, x_hat);
        double[] B_u = Matrix.multiply(B, u);
        x_hat = Matrix.add(F_x, B_u);

        // Predict new covariance: P_k = F * P_k-1 * F^T + Q
        double[][] F_P = Matrix.multiply(F, P);
        double[][] F_P_FT = Matrix.multiply(F_P, Matrix.transpose(F));
        P = Matrix.add(F_P_FT, Q);
    }

    /**
     * Update step: Correct the predicted state with a new measurement.
     * @param z The measurement vector, e.g., from IMU and encoders.
     */
    public void update(double[] z) {
        // Calculate the innovation (residual)
        double[] H_x_hat = Matrix.multiply(H, x_hat);
        double[] y = Matrix.subtract(z, H_x_hat);

        // Calculate the innovation covariance (S)
        double[][] H_P = Matrix.multiply(H, P);
        double[][] H_P_HT = Matrix.multiply(H_P, Matrix.transpose(H));
        double[][] S = Matrix.add(H_P_HT, R);

        // Calculate the Kalman Gain (K)
        double[][] P_HT = Matrix.multiply(P, Matrix.transpose(H));
        double[][] K = Matrix.multiply(P_HT, Matrix.inverse(S));

        // Update the state estimate: x_hat = x_hat + K * y
        double[] K_y = Matrix.multiply(K, y);
        x_hat = Matrix.add(x_hat, K_y);

        // Update the covariance estimate: P = (I - K * H) * P
        double[][] K_H = Matrix.multiply(K, H);
        double[][] I_KH = Matrix.subtract(Matrix.identity(P.length), K_H);
        P = Matrix.multiply(I_KH, P);
    }

    // Getter for the estimated state
    public double[] getEstimatedState() {
        return x_hat;
    }

    // A very basic, non-optimized matrix class for demonstration purposes.
    // In a real application, use a dedicated matrix library.
    private static class Matrix {
        public static double[][] multiply(double[][] A, double[][] B) {
            int rowsA = A.length;
            int colsA = A[0].length;
            int colsB = B[0].length;
            double[][] result = new double[rowsA][colsB];
            for (int i = 0; i < rowsA; i++) {
                for (int j = 0; j < colsB; j++) {
                    for (int k = 0; k < colsA; k++) {
                        result[i][j] += A[i][k] * B[k][j];
                    }
                }
            }
            return result;
        }

        public static double[] multiply(double[][] A, double[] v) {
            int rowsA = A.length;
            int colsA = A[0].length;
            double[] result = new double[rowsA];
            for (int i = 0; i < rowsA; i++) {
                for (int j = 0; j < colsA; j++) {
                    result[i] += A[i][j] * v[j];
                }
            }
            return result;
        }

        public static double[][] transpose(double[][] A) {
            int rows = A.length;
            int cols = A[0].length;
            double[][] result = new double[cols][rows];
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    result[j][i] = A[i][j];
                }
            }
            return result;
        }

        public static double[][] add(double[][] A, double[][] B) {
            int rows = A.length;
            int cols = A[0].length;
            double[][] result = new double[rows][cols];
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    result[i][j] = A[i][j] + B[i][j];
                }
            }
            return result;
        }

        public static double[] add(double[] A, double[] B) {
            int n = A.length;
            double[] result = new double[n];
            for (int i = 0; i < n; i++) {
                result[i] = A[i] + B[i];
            }
            return result;
        }

        public static double[] subtract(double[] A, double[] B) {
            int n = A.length;
            double[] result = new double[n];
            for (int i = 0; i < n; i++) {
                result[i] = A[i] - B[i];
            }
            return result;
        }

        public static double[][] subtract(double[][] A, double[][] B) {
            int rows = A.length;
            int cols = A[0].length;
            double[][] result = new double[rows][cols];
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    result[i][j] = A[i][j] - B[i][j];
                }
            }
            return result;
        }

        // Calculates inverse of a 2x2 matrix. For larger matrices, this is much more complex.
        public static double[][] inverse(double[][] A) {
            if (A.length != 2 || A[0].length != 2) {
                // For a 6x6 matrix, this is much more complex.
                // This is a simplified example.
                throw new IllegalArgumentException("Inverse only supported for 2x2 matrix in this example.");
            }
            double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
            double[][] result = new double[2][2];
            result[0][0] = A[1][1] / det;
            result[0][1] = -A[0][1] / det;
            result[1][0] = -A[1][0] / det;
            result[1][1] = A[0][0] / det;
            return result;
        }

        public static double[][] identity(int size) {
            double[][] result = new double[size][size];
            for (int i = 0; i < size; i++) {
                result[i][i] = 1.0;
            }
            return result;
        }
    }
}
