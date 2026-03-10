package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public final class ClimbConstants {
    // Climb motor configuration
    public static final int CLIMB_MOTOR_ID = 13;
    public static final Current CLIMB_CURRENT_LIMIT = Amps.of(40.0);

    // Manual control voltages
    public static final Voltage MANUAL_UP_VOLTAGE = Volts.of(15.5);
    public static final Voltage MANUAL_DOWN_VOLTAGE = Volts.of(-15.5);

    // CANCoder (absolute encoder) configuration
    public static final int CANCODER_ID = 23;
    public static final boolean CANCODER_INVERTED = false;

    // Limit positions (in rotations, absolute encoder reading)
    // Upper limit: -0.367920 rotations
    // Lower limit: 0.258057 rotations (2.258057 mod 1)
    public static final double UPPER_LIMIT_ROTATIONS = -0.367920;
    public static final double LOWER_LIMIT_ROTATIONS = 0.258057;

    private ClimbConstants() {}
}
