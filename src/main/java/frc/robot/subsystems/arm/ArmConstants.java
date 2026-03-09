package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.arm.ArmConstants.ArmPIDConstants;

public final class ArmConstants {
    public static final Current ARM_CURRENT_LIMIT = Amps.of(20.0);
    public static final Voltage ARM_MAX_VOLTAGE = Volts.of(5.0);

    public static final Current INTAKE_CURRENT_LIMIT = Amps.of(30);
    public static final Voltage INTAKE_MAX_VOLTAGE = Volts.of(23.0);
    public static final Voltage INTAKE_VOLTAGE = Volts.of(6.8);
    public static final double INTAKE_VELOCITY = 1000;
    // The setpoint angle for arm to intake from ground
    public static final Angle ARM_STARTING_ANGLE = Degrees.of(127); // upper
    public static final Angle ARM_INTAKING_ANGLE = Degrees.of(0); // lower

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
            Centimeters.of(38),
            Kilograms.of(2.0),
            DCMotor.getKrakenX60(1),
            48 * 32 / 18,
            // Following data need to be measured on real Robot
            Degrees.of(130), // upper
            Degrees.of(-10), // lower
            Rotation.of(0.181),
            Rotation.of(-0.05),
            22,
            true,
            21,
            true,
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
            0.08, // 0.11
            1.30,
            0.01, // 0.01
            2.1 / Math.toRadians(30), // 6 / Math.toRadians(30)
            RotationsPerSecond.of(1),
            RotationsPerSecondPerSecond.of(5),
            Degrees.of(8));
}
