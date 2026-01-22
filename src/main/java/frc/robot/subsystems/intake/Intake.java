package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConstantsPorts;
import frc.robot.constants.OtherConstants.KrakenX60;
import frc.robot.subsystems.intake.Intake.Position;
import frc.robot.subsystems.intake.Intake.Speed;

public class Intake extends SubsystemBase {
    public enum Speed {
        STOP(0),
        INTAKE(0.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    public enum Position {
        INIT(0), // Initial situation to hold the fuel
        STOWED(0), // STOWED Situation
        INTAKE(0), // INAKE Situation
        HOLD(0); // HOLD and agitate the fuel when bot move

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kPitchReduction = 50.0; // redcution for pitch motor
    private static final AngularVelocity kMaxPitchSpeed = KrakenX60.kFreeSpeed.div(kPitchReduction);
    private static final Angle kPositionTolerance = Degrees.of(5);

    private final TalonFX pitchMotor, rollerMotor;
    private final VoltageOut pitchVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage pitchMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

    private boolean isInit = false;

    public Intake() {
        pitchMotor = new TalonFX(ConstantsPorts.kIntakePitch, ConstantsPorts.kCANivoreCANBus);
        rollerMotor = new TalonFX(ConstantsPorts.kIntakeRollers, ConstantsPorts.kRoboRioCANBus);
        configurePitchMotor();
        configureRollerMotor();
        SmartDashboard.putData(this);
    }

    // PID need change
    private void configurePitchMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive) // change if need
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(70))
                        .withSupplyCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(kPitchReduction))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(kMaxPitchSpeed)
                        .withMotionMagicAcceleration(kMaxPitchSpeed.per(Second)))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(0)
                                .withKI(0)
                                .withKD(0)
                                .withKV(12.0 / kMaxPitchSpeed.in(RotationsPerSecond)) // 12 volts
                        );
        pitchMotor.getConfigurator().apply(config);
    }

    private void configureRollerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(70))
                        .withSupplyCurrentLimitEnable(true));
        rollerMotor.getConfigurator().apply(config);
    }

    private boolean isPositionWithinTolerance() {
        final Angle currentPosition = pitchMotor.getPosition().getValue();
        final Angle targetPosition = pitchMotionMagicRequest.getPositionMeasure();
        return currentPosition.isNear(targetPosition, kPositionTolerance);
    }

    private void setPivotPercentOutput(double percentOutput) {
        pitchMotor.setControl(pitchVoltageRequest.withOutput(Volts.of(percentOutput * 12.0)));
    }

    public void setIntakeSpeed(Speed speed) {
        rollerMotor.setControl(rollerVoltageRequest.withOutput(speed.voltage()));
    }

    public void setIntakePosition(Position position) {
        pitchMotor.setControl(pitchMotionMagicRequest.withPosition(position.angle()));
    }

    public Command intakeCommand() {
        return startEnd(
                () -> {
                    setIntakePosition(Position.INTAKE);
                    setIntakeSpeed(Speed.INTAKE);
                },
                () -> setIntakeSpeed(Speed.STOP));
    }

    public Command holdFuelCommand() {
        return runOnce(() -> setIntakeSpeed(Speed.INTAKE))
                .andThen(Commands.sequence(
                                runOnce(() -> setIntakePosition(Position.HOLD)),
                                Commands.waitUntil(this::isPositionWithinTolerance),
                                runOnce(() -> setIntakePosition(Position.INTAKE)),
                                Commands.waitUntil(this::isPositionWithinTolerance))
                        .repeatedly())
                .handleInterrupt(() -> {
                    setIntakePosition(Position.INTAKE);
                    setIntakeSpeed(Speed.STOP);
                });
    }

    public Command intializingCommand() {
        return Commands.sequence(
                        runOnce(() -> setPivotPercentOutput(0.1)),
                        Commands.waitUntil(
                                () -> pitchMotor.getSupplyCurrent().getValue().in(Amps) > 6),
                        runOnce(() -> {
                            pitchMotor.setPosition(Position.INIT.angle());
                            isInit = true;
                            setIntakePosition(Position.STOWED);
                        }))
                .unless(() -> isInit)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
                "Command",
                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
                null);
        builder.addDoubleProperty(
                "Angle (degrees)", () -> pitchMotor.getPosition().getValue().in(Degrees), null);
        builder.addDoubleProperty(
                "RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(
                "Pivot Supply Current",
                () -> pitchMotor.getSupplyCurrent().getValue().in(Amps),
                null);
        builder.addDoubleProperty(
                "Roller Supply Current",
                () -> rollerMotor.getSupplyCurrent().getValue().in(Amps),
                null);
    }
}
