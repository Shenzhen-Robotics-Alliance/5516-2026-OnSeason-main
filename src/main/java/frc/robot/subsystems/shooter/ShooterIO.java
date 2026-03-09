package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        // Shooter motors array-based inputs
        public boolean[] shootersConnected = new boolean[0];
        public double shooterMotorsAverageVolts = 0.0;
        public double shooterMotorsTotalCurrentAmps = 0.0;
        public double[] shooterMotorsVelocityRPM = new double[0]; // New field for velocity measurements

        // Feeder motors array-based inputs
        public boolean[] feedersConnected = new boolean[0];
        public double feederMotorsAverageVolts = 0.0;
        public double feederMotorsTotalCurrentAmps = 0.0;
        public double[] feederMotorsVelocityRPM = new double[0]; // New field for velocity measurements

        // Subshooter motor inputs (independently controlled subshooter motor)
        // public boolean[] subshootersConnected = new boolean[0];
        // public double[] subshooterMotorsVelocityRPM = new double[0];
    }

    void updateInputs(ShooterIOInputs inputs);

    // Open-loop voltage control (backward compatibility)
    default void setShooterMotorsVoltage(double volts) {}

    default void setFeederMotorsVoltage(double volts) {}

    // Closed-loop velocity control (new feature)
    default void setShooterVelocity(double rpm) {}

    default void setFeederVelocity(double rpm) {}

    // Subshooter inputs (independently controlled subshooter motor)
    // default void updateSubshooterInputs(ShooterIOInputs inputs) {}

    // Shooter control with differential speed (sets both shooter and subshooter)
    // subshooter speed = baseRPM + SUBSHOOTER_RPM_OFFSET
    default void setShooterWithSubshooter(double baseRPM) {}

    // Individual subshooter control
    // default void setSubshooterVelocity(double rpm) {}

    // default void setSubshooterVoltage(double volts) {}
}
