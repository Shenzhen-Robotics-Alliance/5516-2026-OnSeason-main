package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

public final class ArmConstants {
    public static final Current ARM_CURRENT_LIMIT = Amps.of(20.0);
    public static final Voltage ARM_MAX_VOLTAGE = Volts.of(4.0);

    public static final Current INTAKE_CURRENT_LIMIT = Amps.of(30);
    public static final Voltage INTAKE_MAX_VOLTAGE = Volts.of(8.0);
    public static final Voltage INTAKE_VOLTAGE = Volts.of(5.0);

    // The setpoint angle for arm to intake from ground
    public static final Angle ARM_INTAKING_ANGLE = Degrees.of(12);
    public static final Angle ARM_STARTING_ANGLE = Degrees.of(90);

    public record ArmHardwareConstants(
            Distance ARM_COM_LENGTH,
            Mass ARM_MASS,
            DCMotor ARM_GEARBOX,
            double ARM_GEARING_REDUCTION,
            Angle ARM_UPPER_HARD_LIMIT,
            Angle ARM_LOWER_HARD_LIMIT,
            Angle ABSOLUTE_ENCODER_READING_AT_UPPER_LIM,
            Angle ABSOLUTE_ENCODER_READING_AT_LOWER_LIM,
            //     int ABSOLUTE_ENCODER_CHANNEL,
            int ABSOLUTE_ENCODER_ID,
            boolean ABSOLUTE_ENCODER_INVERTED,
            int ARM_MOTOR_ID,
            boolean ARM_MOTOR_INVERTED,
            int INTAKE_MOTOR_ID,
            boolean INTAKE_MOTOR_INVERTED) {}

    public static final ArmHardwareConstants HARDWARE_CONSTANTS = new ArmHardwareConstants(
            Centimeters.of(33),
            Kilograms.of(3.0),
            DCMotor.getKrakenX60(1),
            45 * 32 / 18,
            // Following data need to be measured on real Robot
            Degrees.of(90),
            Degrees.of(0),
            Rotation.of(-0.186),
            Rotation.of(0),
            22,
            false,
            21,
            false,
            20,
            true);

    public record ArmPIDConstants(
            double kS,
            double kG,
            double kV,
            double kA,
            double kP,
            AngularVelocity VELOCTIY_CONSTRAIN,
            AngularAcceleration ACCELERATION_CONSTRAIN,
            Angle TOLERANCE) {}

    public static final ArmPIDConstants PID_CONSTANTS = new ArmPIDConstants(
            0.05,
            0.08,
            1.51,
            0.01,
            6.0 / Math.toRadians(30),
            RotationsPerSecond.of(1),
            RotationsPerSecondPerSecond.of(5),
            Degrees.of(3));
}
