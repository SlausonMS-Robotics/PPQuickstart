package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;

public class other_helpers {

    // ---- PID Controller variables ----
    private double p, i, d;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // ---- Moving Average variables ----
    private ArrayList<Double> readings = new ArrayList<>();
    private int movingAverageSize = 2; // Default size

    /**
     * Initializes the PID controller with the given coefficients.
     *
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     */
    public void initPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
        resetPID();
    }

    /**
     * Calculates the PID output based on the current and target states.
     *
     * @param currentState The current measurement
     * @param targetState  The desired measurement
     * @return The calculated PID correction
     */
    public double updatePID(double currentState, double targetState) {
        double error = targetState - currentState;
        integralSum += error * timer.seconds();

        // Handle the case where timer.seconds() is 0 on the first loop
        if (timer.seconds() == 0) {
            timer.reset();
            // cannot calculate derivative on first loop
            return (p * error) + (i * integralSum);
        }

        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (p * error) + (i * integralSum) + (d * derivative);
    }

    /**
     * Resets the PID controller's internal state.
     */
    public void resetPID() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    // ---- Moving Average Methods ----

    /**
     * Initializes the moving average with a specific size.
     *
     * @param size The number of readings to average.
     */
    public void initMovingAverage(int size) {
        this.movingAverageSize = size;
        this.readings.clear();
    }

    /**
     * Adds a new reading and returns the new average.
     *
     * @param reading The new measurement to add.
     * @return The new average of the readings.
     */
    public double updateAndGetAverage(double reading) {
        readings.add(reading);
        if (readings.size() > movingAverageSize) {
            readings.remove(0);
        }

        if (readings.isEmpty()) {
            return 0.0;
        }

        double sum = 0;
        for (Double r : readings) {
            sum += r;
        }
        return sum / readings.size();
    }

    /**
     * Clears all readings from the moving average.
     */
    public void clearReadings() {
        readings.clear();
    }

    // --- Flywheel Shooter Utilities ---
    public static class FlywheelShooter {

        // --- Linear Model Constants ---
        //
        private static final double RPM_PER_METER = 200; // The slope of the line (how much RPM to add per meter)
        private static final double BASE_RPM = 3400;     // The base RPM at 0 meters (y-intercept)

        /**
         * Get required flywheel RPM for a given shot distance (m) using a linear model.
         * A linear model is often more accurate than a simple physics model because
         * it can be tuned to account for real-world factors like air resistance and energy loss.
         */
        public static double getRPMForDistance(double rangeMeters) {
            if (rangeMeters <= 2) return BASE_RPM;
            return BASE_RPM + (rangeMeters * RPM_PER_METER);
        }

        /** Predict horizontal range (m) for a given flywheel RPM. */
        public static double getDistanceForRPM(double rpm) {
            if (rpm <= BASE_RPM) return 0;
            return (rpm - BASE_RPM) / RPM_PER_METER;
        }
    }

}
