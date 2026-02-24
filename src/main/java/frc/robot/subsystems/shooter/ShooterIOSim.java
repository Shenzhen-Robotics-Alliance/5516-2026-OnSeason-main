package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterContants.*;

import edu.wpi.first.math.MathUtil;

public class ShooterIOSim implements ShooterIO {
    // Simple simulation state for shooter motors
    private final double[] shooterVelocitiesRPM;
    private final double[] shooterAppliedVolts;
    private final double[] shooterTargetRPM;

    // Simple simulation state for feeder motors
    private final double[] feederVelocitiesRPM;
    private final double[] feederAppliedVolts;
    private final double[] feederTargetRPM;

    // Track current control mode
    private boolean shooterVelocityControl = false;
    private boolean feederVelocityControl = false;

    // PID controllers for velocity control
    private static final double SHOOTER_KP = 0.1;
    private static final double SHOOTER_KI = 0.0;
    private static final double SHOOTER_KD = 0.0;

    private static final double FEEDER_KP = 0.1;
    private static final double FEEDER_KI = 0.0;
    private static final double FEEDER_KD = 0.0;

    // Simulation parameters
    private static final double MOTOR_RESISTANCE = 0.05; // Ohms
    private static final double MOTOR_KV = 0.012; // Volts per RPM
    private static final double MOTOR_INERTIA = 0.001; // kg·m²
    private static final double SIMULATION_DT = 0.02; // 20ms update period

    // PID accumulators
    private double shooterIntegral = 0.0;
    private double feederIntegral = 0.0;
    private double shooterPrevError = 0.0;
    private double feederPrevError = 0.0;

    public ShooterIOSim() {
        // Initialize shooter simulation
        int shooterCount = SHOOTERHARDWARE_CONSTANTS.shooterMotorIDs().length;
        shooterVelocitiesRPM = new double[shooterCount];
        shooterAppliedVolts = new double[shooterCount];
        shooterTargetRPM = new double[shooterCount];

        for (int i = 0; i < shooterCount; i++) {
            shooterVelocitiesRPM[i] = 0.0;
            shooterAppliedVolts[i] = 0.0;
            shooterTargetRPM[i] = 0.0;
        }

        // Initialize feeder simulation
        int feederCount = SHOOTERHARDWARE_CONSTANTS.feederMotorIDs().length;
        feederVelocitiesRPM = new double[feederCount];
        feederAppliedVolts = new double[feederCount];
        feederTargetRPM = new double[feederCount];

        for (int i = 0; i < feederCount; i++) {
            feederVelocitiesRPM[i] = 0.0;
            feederAppliedVolts[i] = 0.0;
            feederTargetRPM[i] = 0.0;
        }
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update shooter inputs
        int shooterCount = shooterVelocitiesRPM.length;
        inputs.shootersConnected = new boolean[shooterCount];
        inputs.shooterMotorsVelocityRPM = new double[shooterCount];

        double shooterTotalVolts = 0.0;
        double shooterTotalCurrent = 0.0;

        for (int i = 0; i < shooterCount; i++) {
            // All motors are "connected" in simulation
            inputs.shootersConnected[i] = true;

            // Apply velocity control if enabled
            if (shooterVelocityControl && i == 0) { // Only control leader in simulation
                double error = shooterTargetRPM[i] - shooterVelocitiesRPM[i];

                // PID calculation
                shooterIntegral += error * SIMULATION_DT;
                double derivative = (error - shooterPrevError) / SIMULATION_DT;
                shooterPrevError = error;

                double controlOutput = SHOOTER_KP * error + SHOOTER_KI * shooterIntegral + SHOOTER_KD * derivative;
                controlOutput = MathUtil.clamp(controlOutput, -12.0, 12.0);
                shooterAppliedVolts[i] = controlOutput;
            }

            // Simple motor model: velocity = (voltage - backEMF) / resistance * KV
            double backEMF = shooterVelocitiesRPM[i] * MOTOR_KV;
            double effectiveVoltage = shooterAppliedVolts[i] - backEMF;
            double acceleration = (effectiveVoltage / MOTOR_RESISTANCE) * MOTOR_KV / MOTOR_INERTIA;

            // Update velocity
            shooterVelocitiesRPM[i] += acceleration * SIMULATION_DT;
            shooterVelocitiesRPM[i] = Math.max(0, shooterVelocitiesRPM[i]); // Can't go negative

            // Set output
            inputs.shooterMotorsVelocityRPM[i] = shooterVelocitiesRPM[i];

            // Calculate current: I = (V - backEMF) / R
            double current = Math.abs(effectiveVoltage) / MOTOR_RESISTANCE;
            shooterTotalCurrent += current;

            // Track applied voltage
            shooterTotalVolts += shooterAppliedVolts[i];
        }

        inputs.shooterMotorsAverageVolts = shooterCount > 0 ? shooterTotalVolts / shooterCount : 0.0;
        inputs.shooterMotorsTotalCurrentAmps = shooterTotalCurrent;

        // Update feeder inputs
        int feederCount = feederVelocitiesRPM.length;
        inputs.feedersConnected = new boolean[feederCount];
        inputs.feederMotorsVelocityRPM = new double[feederCount];

        double feederTotalVolts = 0.0;
        double feederTotalCurrent = 0.0;

        for (int i = 0; i < feederCount; i++) {
            // All motors are "connected" in simulation
            inputs.feedersConnected[i] = true;

            // Apply velocity control if enabled
            if (feederVelocityControl && i == 0) { // Only control leader in simulation
                double error = feederTargetRPM[i] - feederVelocitiesRPM[i];

                // PID calculation
                feederIntegral += error * SIMULATION_DT;
                double derivative = (error - feederPrevError) / SIMULATION_DT;
                feederPrevError = error;

                double controlOutput = FEEDER_KP * error + FEEDER_KI * feederIntegral + FEEDER_KD * derivative;
                controlOutput = MathUtil.clamp(controlOutput, -12.0, 12.0);
                feederAppliedVolts[i] = controlOutput;
            }

            // Simple motor model: velocity = (voltage - backEMF) / resistance * KV
            double backEMF = feederVelocitiesRPM[i] * MOTOR_KV;
            double effectiveVoltage = feederAppliedVolts[i] - backEMF;
            double acceleration = (effectiveVoltage / MOTOR_RESISTANCE) * MOTOR_KV / MOTOR_INERTIA;

            // Update velocity
            feederVelocitiesRPM[i] += acceleration * SIMULATION_DT;
            feederVelocitiesRPM[i] = Math.max(0, feederVelocitiesRPM[i]); // Can't go negative

            // Set output
            inputs.feederMotorsVelocityRPM[i] = feederVelocitiesRPM[i];

            // Calculate current: I = (V - backEMF) / R
            double current = Math.abs(effectiveVoltage) / MOTOR_RESISTANCE;
            feederTotalCurrent += current;

            // Track applied voltage
            feederTotalVolts += feederAppliedVolts[i];
        }

        inputs.feederMotorsAverageVolts = feederCount > 0 ? feederTotalVolts / feederCount : 0.0;
        inputs.feederMotorsTotalCurrentAmps = feederTotalCurrent;
    }

    @Override
    public void setShooterMotorsVoltage(double volts) {
        shooterVelocityControl = false;
        shooterIntegral = 0.0;
        shooterPrevError = 0.0;

        // Apply voltage to all shooter motors (simplified simulation)
        for (int i = 0; i < shooterVelocitiesRPM.length; i++) {
            shooterAppliedVolts[i] = volts;
        }
    }

    @Override
    public void setFeederMotorsVoltage(double volts) {
        feederVelocityControl = false;
        feederIntegral = 0.0;
        feederPrevError = 0.0;

        // Apply voltage to all feeder motors (simplified simulation)
        for (int i = 0; i < feederVelocitiesRPM.length; i++) {
            feederAppliedVolts[i] = volts;
        }
    }

    @Override
    public void setShooterVelocity(double rpm) {
        shooterVelocityControl = true;
        shooterIntegral = 0.0;
        shooterPrevError = 0.0;

        // Set target RPM for all shooter motors
        for (int i = 0; i < shooterVelocitiesRPM.length; i++) {
            shooterTargetRPM[i] = rpm;
        }
    }

    @Override
    public void setFeederVelocity(double rpm) {
        feederVelocityControl = true;
        feederIntegral = 0.0;
        feederPrevError = 0.0;

        // Set target RPM for all feeder motors
        for (int i = 0; i < feederVelocitiesRPM.length; i++) {
            feederTargetRPM[i] = rpm;
        }
    }
}
