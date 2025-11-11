/* Copyright (c) 2019-2025 ParkCircus Productions
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @license MIT
 */

package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A Robust **PID (Proportional-Integral-Derivative) Controller** class designed
 * for high-performance control systems, commonly used in robotics for achieving
 * a target setpoint.
 *
 * <p>This controller includes several advanced features to improve stability and performance:</p>
 * <ul>
 * <li>**Feedforward (Kf):** Allows compensating for steady-state system requirements (e.g., gravity or friction) by predicting the power needed for a target velocity.</li>
 * <li>**Low-Pass Filter (LPF) on Derivative:** Smooths the derivative term (Kd) to prevent noisy sensor readings from causing rapid, jittery output changes.</li>
 * <li>**Conditional Integration (Anti-Windup):** Prevents the integral sum from "winding up" when the output is saturated or when the error is large, improving recovery time.</li>
 * <li>**Continuous Input:** Supports control over cyclical systems (e.g., headings from 0 to 360 degrees).</li>
 * </ul>
 *
 * <p>The controller is designed to be time-based, calculating the delta time (dt)
 * on each call to ensure proper integral and derivative term scaling, regardless
 * of the calling frequency.</p>
 */
public class PIDController {

    // PID Gains
    private double Kp, Ki, Kd, Kf;
    private double setpoint;

    // PID State
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private long lastTimeNanos = 0;
    private double integralLimit = Double.POSITIVE_INFINITY; // Anti-Windup Limit

    // Derivative Filter variables (First-order Low-Pass Filter)
    private double derivativeFilterTimeConstant = 0.0; // Time constant in seconds (Tau). Set to 0.0 for no filter.
    private double lastFilteredError = 0.0;

    // Threshold for Conditional Integration (e.g., only integrate if error is small)
    private double integralErrorThreshold = Double.POSITIVE_INFINITY;

    // Continuous Input variables
    private boolean isContinuous = false;
    private double minInput = 0.0;
    private double maxInput = 0.0;

    /**
     * Initializes the Robust PID controller.
     *
     * <p>Initializes all gains and sets the starting target value.</p>
     *
     * @param Kp Proportional gain. Typically the largest gain, responsible for response to current error.
     * @param Ki Integral gain. Used to eliminate steady-state error over time.
     * @param Kd Derivative gain. Used to dampen oscillation and respond to the rate of error change.
     * @param Kf Feedforward gain. Used to add a term proportional to the target velocity or acceleration.
     * @param initialSetpoint The initial target value (the desired final state).
     */
    public PIDController(double Kp, double Ki, double Kd, double Kf, double initialSetpoint) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.setpoint = initialSetpoint;
        this.lastTimeNanos = System.nanoTime();
    }

    /**
     * Calculates the PID output given the current measured value and target velocity.
     *
     * <p>This is the core function of the controller. It calculates the error,
     * updates the integral sum (with anti-windup), computes the filtered derivative,
     * and combines all terms (P, I, D, F) to produce the final control output.</p>
     *
     * @param currentValue The sensor measurement (e.g., Limelight RangeZ, motor position, or IMU angle).
     * @param targetVelocity The expected velocity of the setpoint (used for Feedforward). Use {@code 0.0} if not using a Feedforward term.
     * @return The calculated control output, typically ranging from -1.0 to 1.0 for motor power.
     */
    public double calculate(double currentValue, double targetVelocity) {
        long currentTimeNanos = System.nanoTime();
        // Calculate delta time (dt) in seconds
        double dt = (currentTimeNanos - lastTimeNanos) / 1_000_000_000.0;
        lastTimeNanos = currentTimeNanos;

        // --- 1. Error Calculation ---
        double error = setpoint - currentValue;

        // Handle continuous input
        if (isContinuous && maxInput > minInput) {
            double range = maxInput - minInput;
            if (error > range / 2.0) {
                error -= range;
            } else if (error < -range / 2.0) {
                error += range;
            }
        }

        // --- 2. Feedforward Term ---
        double feedforwardTerm = Kf * targetVelocity;

        // --- 3. Proportional Term ---
        double proportionalTerm = Kp * error;

        // --- 4. Integral Term (Conditional Integration Anti-Windup) ---
        double integralTerm = 0.0;
        if (Ki != 0.0 && dt > 0) {
            // Conditional Integration: Only integrate if the error is within the threshold.
            if (Math.abs(error) < integralErrorThreshold) {
                integralSum += error * dt;

                // Anti-windup clamping
                integralSum = Math.min(integralSum, integralLimit);
                integralSum = Math.max(integralSum, -integralLimit);
            }
            integralTerm = Ki * integralSum;
        }

        // --- 5. Derivative Term (with Low-Pass Filter) ---
        double derivativeTerm = 0.0;
        if (Kd != 0.0 && dt > 0) {

            // Calculate raw error rate
            double rawDerivative = (error - lastError) / dt;

            // Low-Pass Filter (LPF) application: Y_new = Y_old + alpha * (X_new - Y_old)
            double alpha = (derivativeFilterTimeConstant == 0) ? 1.0 : dt / (derivativeFilterTimeConstant + dt);
            double filteredDerivative = lastFilteredError + alpha * (rawDerivative - lastFilteredError);

            derivativeTerm = Kd * filteredDerivative;
            lastFilteredError = filteredDerivative;
        }

        // Update last error for the next iteration
        lastError = error;

        // 6. Combined PIDF Output
        return proportionalTerm + integralTerm + derivativeTerm + feedforwardTerm;
    }

    // --- Configuration Methods ---
    /**
     * Sets a new target value for the controller to track.
     *
     * @param newSetpoint The new desired target value.
     */
    public void setSetpoint(double newSetpoint) {
        this.setpoint = newSetpoint;
    }

    /**
     * Sets the time constant (Tau) for the derivative low-pass filter (in seconds).
     *
     * <p>A higher time constant results in more aggressive filtering, slowing the
     * reaction to noise but also potentially delaying the response to actual rate changes.
     * Setting this to {@code 0.0} disables the filter.</p>
     *
     * @param timeConstant The filter time constant (Tau) in seconds.
     */
    public void setDerivativeFilter(double timeConstant) {
        this.derivativeFilterTimeConstant = timeConstant;
    }

    /**
     * Sets the maximum absolute value for the integral accumulator (Anti-Windup).
     *
     * <p>This limit prevents the integral term from growing too large when the control
     * output is saturated, resulting in faster recovery time when the system returns
     * to the controllable range.</p>
     *
     * @param limit The maximum absolute value for {@code integralSum}.
     */
    public void setIntegralLimit(double limit) {
        this.integralLimit = Math.abs(limit);
    }

    /**
     * Sets the maximum error value for integration to occur (Conditional Integration).
     *
     * <p>If the absolute error is larger than this threshold, the integral sum will
     * not accumulate. This is an additional anti-windup measure, often used to prevent
     * the integral from accumulating while the robot is far from the target.</p>
     *
     * @param threshold The maximum absolute error to allow integration.
     */
    public void setIntegralErrorThreshold(double threshold) {
        this.integralErrorThreshold = Math.abs(threshold);
    }

    /**
     * Enables continuous input handling for cyclical systems (e.g., angle control).
     *
     * <p>When enabled, the controller will choose the shortest path around the input range
     * when calculating the error (e.g., going from 350 degrees to 10 degrees is an error of 20, not -340).</p>
     *
     * @param minInput The minimum value of the continuous range (e.g., 0 for a circle).
     * @param maxInput The maximum value of the continuous range (e.g., 360 for a circle).
     */
    public void enableContinuousInput(double minInput, double maxInput) {
        this.isContinuous = true;
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    /**
     * Resets the internal state of the controller.
     *
     * <p>This clears the accumulated integral sum, the last error, and the filtered
     * derivative value. This should be called whenever the controller is enabled
     * or the robot's control task is fundamentally changed to prevent carry-over from
     * a previous control loop.</p>
     */
    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
        lastFilteredError = 0.0;
        lastTimeNanos = System.nanoTime();
    }
}