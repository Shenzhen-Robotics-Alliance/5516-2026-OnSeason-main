package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    class ClimbInputs {
        boolean hardwareConnected = false;
        double climbCurrentAmps = 0.0;
        double climbAbsolutePosition = 0.0;
        double supportCurrentAmps = 0.0;
        double supportAbsolutePosition = 0.0;
    }

    void updateInputs(ClimbInputs inputs);

    default void setMotorOutput(double volts) {}

    default void setSupportMotorOutput(double volts) {}
}
