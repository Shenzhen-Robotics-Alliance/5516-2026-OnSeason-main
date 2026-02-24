package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.AlertsManager;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    // Hardware interface
    private final ArmIO io;
    private final ArmIO.ArmInputs inputs;

    // Controllers
    private final ArmFeedforward feedforwardController;
    private final PIDController feedbackController;
    private final TrapezoidProfile profile;

    // Alert
    private final Alert armHardwareFaultsAlert;
    private final Alert armNotCalibratedAlert;
    private final Alert armAbsoluteEncoderDisconnectedAlert;

    /** Debounce for hardware faults */
    private final Debouncer hardwareFaultDebouncer;
    /** Whether there is a hardware fault in the motor */
    private boolean hardwareFaultDetected;
    /** Whether the encoder has been calibrated since booting */
    private boolean encoderCalibrated;
    /** The offset of the relative encoder (relative - actual) */
    private Rotation2d relativeEncoderOffset;

    /** The current state */
    private TrapezoidProfile.State currentStateRad;

    /** The desired angle */
    private Angle setpoint;

    public Arm(ArmIO io) {
        this.io = io;
        inputs = new ArmIO.ArmInputs();

        this.feedforwardController =
                new ArmFeedforward(PID_CONSTANTS.kS(), PID_CONSTANTS.kG(), PID_CONSTANTS.kV(), PID_CONSTANTS.kA());
        this.feedbackController = new PIDController(PID_CONSTANTS.kP(), 0, 0);
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                PID_CONSTANTS.VELOCTIY_CONSTRAIN().in(RadiansPerSecond),
                PID_CONSTANTS.ACCELERATION_CONSTRAIN().in(RadiansPerSecondPerSecond)));
        this.armHardwareFaultsAlert = AlertsManager.create("Arm hardware faults detected!", AlertType.kError);
        this.armNotCalibratedAlert =
                AlertsManager.create("Arm encoder not calibrated! (Arm fault highly likely)", AlertType.kError);
        this.armAbsoluteEncoderDisconnectedAlert = AlertsManager.create(
                "Arm encoder calibrated, but absolute encoder disconnected. (Check it post-match).",
                AlertType.kWarning);
        this.hardwareFaultDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kRising);

        this.armHardwareFaultsAlert.set(false);
        this.armNotCalibratedAlert.set(false);
        this.armAbsoluteEncoderDisconnectedAlert.set(false);

        currentStateRad = new TrapezoidProfile.State(
                HARDWARE_CONSTANTS.ARM_UPPER_HARD_LIMIT().in(Radians), 0.0);
        setpoint = ARM_STARTING_ANGLE;

        hardwareFaultDetected = false;
        encoderCalibrated = false;

        /**
         * Calibrate the relative encoder. Assumes that the arm starts at upper limit during boot. The calibration will
         * be overwritten if the absolute encoder is available. Offset = Relative Angle - Actual Angle, where Relative
         * Angle is 0 when booting.
         */
        relativeEncoderOffset = Rotation2d.kZero.minus(new Rotation2d(HARDWARE_CONSTANTS.ARM_UPPER_HARD_LIMIT()));

        io.setArmMotorBrake(true);
        io.setIntakeMotorBrake(true);
    }

    /**
     * Calibrates the relative encoder offset using the absolute encoder reading real = relative - offset, so offset =
     * relative - real
     */
    private void calibrateEncoders(Rotation2d absoluteEncoderAngle) {
        relativeEncoderOffset = Rotation2d.fromRadians(
                        inputs.relativeEncoderAngledRad / HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION())
                .minus(absoluteEncoderAngle);
        encoderCalibrated = true;
    }

    /**
     * Sets the arm motor to idle
     *
     * <p><b>Note: This does not unlock the motors if they are set to brake mode
     */
    private void executeIdle() {
        io.setArmMotorOutput(Volts.zero());
        io.setIntakeMotorOutput(Volts.zero());
        currentStateRad = new TrapezoidProfile.State(getArmAngle().getRadians(), 0);
        previousVelocityRadPerSec = 0.0;
        logControlLoops(0, 0, 0);
    }

    private double previousVelocityRadPerSec = 0.0;

    /** Runs the control loops on the arm to achieve the setpoint */
    private void executeControlLoops(double dtSeconds) {
        dtSeconds = MathUtil.clamp(dtSeconds, 0, 0.1);
        TrapezoidProfile.State goalState = new TrapezoidProfile.State(setpoint.in(Radians), 0);
        currentStateRad = profile.calculate(dtSeconds, currentStateRad, goalState);

        double accelerationRadPerSecSq =
                (currentStateRad.velocity - previousVelocityRadPerSec) / Robot.defaultPeriodSecs;
        previousVelocityRadPerSec = currentStateRad.velocity;

        double feedforwardVolts = feedforwardController.calculate(
                getArmAngle().getRadians(), currentStateRad.velocity, accelerationRadPerSecSq);

        double feedbackVolts = feedbackController.calculate(getArmAngle().getRadians(), currentStateRad.position);

        Voltage outputVolts = Volts.of(MathUtil.clamp(
                feedforwardVolts + feedbackVolts, -ARM_MAX_VOLTAGE.in(Volts), ARM_MAX_VOLTAGE.in(Volts)));
        io.setArmMotorOutput(outputVolts);
        logControlLoops(feedforwardVolts, feedbackVolts, outputVolts.in(Volts));
    }

    private void logControlLoops(double feedforwardVolts, double feedbackVolts, double outputVolts) {
        Logger.recordOutput("Arm/PID/Feedforward Volts", feedforwardVolts);
        Logger.recordOutput("Arm/PID/Feedback Volts", feedbackVolts);
        Logger.recordOutput("Arm/PID/Requseted Output Volts", outputVolts);
    }

    private double previousTimeSeconds = Timer.getTimestamp();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        executeControlLoops(Timer.getTimestamp() - previousTimeSeconds);
        if (DriverStation.isDisabled()) executeIdle();
        previousTimeSeconds = Timer.getTimestamp();

        // Calibrates encoder if needed when the robot is disable and the motor is connected
        if (!encoderCalibrated && DriverStation.isDisabled() && inputs.armMotorConnected)
            inputs.absoluteEncoderAngle.ifPresent(this::calibrateEncoders);

        // Updates Alerts
        hardwareFaultDetected = hardwareFaultDebouncer.calculate(!inputs.armMotorConnected);
        if (hardwareFaultDetected) encoderCalibrated = false;

        armHardwareFaultsAlert.set(hardwareFaultDetected);
        armNotCalibratedAlert.set(!encoderCalibrated);
        armAbsoluteEncoderDisconnectedAlert.set(encoderCalibrated && inputs.absoluteEncoderAngle.isEmpty());

        Logger.recordOutput("Arm/Setpoint (Degrees)", setpoint.in(Degrees));
        Logger.recordOutput("Arm/Current State Positon(Degrees)", Math.toDegrees(currentStateRad.position));
        Logger.recordOutput("Arm/MeasuredAngle (Degrees)", getArmAngle().getDegrees());

        Logger.recordOutput(
                "Arm/Current State Velocity (Degrees Per Second)", Math.toDegrees(currentStateRad.velocity));
        Logger.recordOutput("Arm/Measured Velocity (Degrees Per Second)", Math.toDegrees(getVelocityRadPerSec()));
        Logger.recordOutput("Arm/AtReference", atReference());
    }

    /**
     * Whether the arm's profile state is close enough to its setpoint.
     *
     * <p>Note that this does not reflect whether the mechanism is actually at its setpoint.
     *
     * @return <code>true</code> if there is a setpoint and the profile is close enough to the goal, <code>false</code>
     *     otherwise.
     */
    public boolean atReference() {
        return atReference(this.setpoint);
    }

    public boolean atReference(Angle setpoint) {
        double errorRad = Rotation2d.fromRadians(currentStateRad.position)
                .minus(new Rotation2d(setpoint))
                .getRadians();
        // errorRad smaller than the tolerance value means the setpoint and the profile is close enough to the goal
        return Math.abs(errorRad) < PID_CONSTANTS.TOLERANCE().in(Radians);
    }

    /**
     * Whether the mechanism is actually close enough to its setpoint.
     *
     * @return <code>true</code> if there is a setpoint and the measured arm angle is close enough to the goal, <code>
     *     false</code> otherwise.
     */
    public boolean trulyAtReference() {
        return trulyAtReference(this.setpoint);
    }

    public boolean trulyAtReference(Angle setpoint) {
        double errorRad = getArmAngle().minus(new Rotation2d(setpoint)).getRotations();
        return Math.abs(errorRad) < PID_CONSTANTS.TOLERANCE().in(Radians);
    }
    /** Request the arm to move to a given setpoint */
    public void requestPosition(Angle setpoint) {
        this.setpoint = setpoint;
    }

    /** @return the measured arm angle, where zero is horizontally forward */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromRadians(inputs.relativeEncoderAngledRad / HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION())
                .minus(relativeEncoderOffset);
    }

    public double getVelocityRadPerSec() {
        return inputs.relativeEncoderVelocityRadPerSec / HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION();
    }

    public Rotation2d getProfileCurrentState() {
        return Rotation2d.fromRadians(currentStateRad.position);
    }

    /** Sets the brake mode of the arm motor. */
    public void setArmMotorBrake(boolean brakeModeEnable) {
        io.setArmMotorBrake(brakeModeEnable);
    }

    /** Sets the brake mode of the arm motor. */
    public void setIntakeMotorBrake(boolean brakeModeEnable) {
        io.setIntakeMotorBrake(brakeModeEnable);
    }

    /**
     * Moves to a given angle, cancels the setpoint when reached.
     *
     * <p>When brake mode is on, the arm will still hold at that angle.
     *
     * <p><b>Note: This command finishes automatically when the setpoint is reached, causing the default command to
     * schedule. </b>
     */
    public Command moveToPosition(Angle setpoint) {
        return run(() -> requestPosition(setpoint)).until(this::atReference);
    }

    /** Dropping down the arm to intake the fuel */
    public Command armDroppingCommand() {
        return moveToPosition(ARM_INTAKING_ANGLE);
    }

    /** Upright the arm to the starting position */
    public Command armUprightCommand() {
        return moveToPosition(ARM_STARTING_ANGLE);
    }

    public Command intakeCommand() {
        return run(() -> io.setIntakeMotorOutput(INTAKE_VOLTAGE));
    }

    public Command intakeIdleCommand() {
        return run(() -> io.setIntakeMotorOutput(Volts.zero()));
    }
}
