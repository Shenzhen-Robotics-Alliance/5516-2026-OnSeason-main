package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertsManager;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    // Hardware interface
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;

    private double autoShooterRPM = 0;
    private double distToHub = 0;

    // Dynamic alerts based on motor count
    private Alert[] shooterMotorAlerts;
    private Alert[] feederMotorAlerts;
    // private Alert[] subshooterMotorAlerts;

    public Shooter(ShooterIO io) {
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();

        // Initialize alerts arrays (will be populated in periodic based on actual motor count)
        shooterMotorAlerts = new Alert[0];
        feederMotorAlerts = new Alert[0];
        //  subshooterMotorAlerts = new Alert[0];
    }

    public boolean hardwareOK() {
        // Check all shooter motors
        if (inputs.shootersConnected != null) {
            for (boolean connected : inputs.shootersConnected) {
                if (!connected) return false;
            }
        }

        // Check all feeder motors
        if (inputs.feedersConnected != null) {
            for (boolean connected : inputs.feedersConnected) {
                if (!connected) return false;
            }
        }

        return true;
    }

    @Override
    public void periodic() {
        // Update inputs from IO and AdvantageKit
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Initialize or update alerts based on current motor count
        updateAlerts();

        if (DriverStation.isDisabled()) executeIdle();

        // Update alerts
        if (shooterMotorAlerts.length > 0 && inputs.shootersConnected != null) {
            for (int i = 0; i < Math.min(shooterMotorAlerts.length, inputs.shootersConnected.length); i++) {
                if (shooterMotorAlerts[i] != null) {
                    shooterMotorAlerts[i].set(!inputs.shootersConnected[i]);
                }
            }
        }

        if (feederMotorAlerts.length > 0 && inputs.feedersConnected != null) {
            for (int i = 0; i < Math.min(feederMotorAlerts.length, inputs.feedersConnected.length); i++) {
                if (feederMotorAlerts[i] != null) {
                    feederMotorAlerts[i].set(!inputs.feedersConnected[i]);
                }
            }
        }

        // Log motor connections
        if (inputs.shootersConnected != null) {
            for (int i = 0; i < inputs.shootersConnected.length; i++) {
                Logger.recordOutput("ShooterMotor" + (i + 1) + "/connected", inputs.shootersConnected[i]);
            }
        }

        if (inputs.feedersConnected != null) {
            for (int i = 0; i < inputs.feedersConnected.length; i++) {
                Logger.recordOutput("FeederMotor" + (i + 1) + "/connected", inputs.feedersConnected[i]);
            }
        }

        // Log aggregated data
        Logger.recordOutput("Shooter/AverageVolts", inputs.shooterMotorsAverageVolts);
        Logger.recordOutput("Shooter/TotalCurrentAmps", inputs.shooterMotorsTotalCurrentAmps);
        Logger.recordOutput("Feeder/AverageVolts", inputs.feederMotorsAverageVolts);
        Logger.recordOutput("Feeder/TotalCurrentAmps", inputs.feederMotorsTotalCurrentAmps);
        Logger.recordOutput("Shooter/AutoTargetRPM", autoShooterRPM);
        Logger.recordOutput("Shooter/DistanceToHub", distToHub);

        // Log velocity data if available
        if (inputs.shooterMotorsVelocityRPM != null && inputs.shooterMotorsVelocityRPM.length > 0) {
            Logger.recordOutput(
                    "Shooter/AverageVelocityRPM", calculateAverageVelocity(inputs.shooterMotorsVelocityRPM));
        }

        if (inputs.feederMotorsVelocityRPM != null && inputs.feederMotorsVelocityRPM.length > 0) {
            Logger.recordOutput("Feeder/AverageVelocityRPM", calculateAverageVelocity(inputs.feederMotorsVelocityRPM));
        }
    }

    private void updateAlerts() {
        // Create alerts for shooter motors if needed
        if (inputs.shootersConnected != null && shooterMotorAlerts.length != inputs.shootersConnected.length) {
            shooterMotorAlerts = new Alert[inputs.shootersConnected.length];
            for (int i = 0; i < shooterMotorAlerts.length; i++) {
                shooterMotorAlerts[i] =
                        AlertsManager.create("ShooterMotor" + (i + 1) + " hardware detected!", AlertType.kError);
            }
        }

        // Create alerts for feeder motors if needed
        if (inputs.feedersConnected != null && feederMotorAlerts.length != inputs.feedersConnected.length) {
            feederMotorAlerts = new Alert[inputs.feedersConnected.length];
            for (int i = 0; i < feederMotorAlerts.length; i++) {
                feederMotorAlerts[i] =
                        AlertsManager.create("FeederMotor" + (i + 1) + " hardware detected!", AlertType.kError);
            }
        }
    }

    private double calculateAverageVelocity(double[] velocities) {
        if (velocities == null || velocities.length == 0) return 0.0;

        double sum = 0.0;
        int count = 0;
        for (double velocity : velocities) {
            sum += velocity;
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    private double calculateAverageAbsoluteVelocity(double[] velocities) {
        if (velocities == null || velocities.length == 0) return 0.0;

        double sum = 0.0;
        int count = 0;
        for (double velocity : velocities) {
            sum += Math.abs(velocity);
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    private void setShooterMotorVolts(double shooterMotorVolts) {
        if (!hardwareOK()) shooterMotorVolts = 0;
        io.setShooterMotorsVoltage(shooterMotorVolts);
    }

    private void setFeederMotorVolts(double feederMotorVolts) {
        if (!hardwareOK()) feederMotorVolts = 0;
        io.setFeederMotorsVoltage(feederMotorVolts);
    }

    private void setShooterVelocity(double rpm) {
        if (!hardwareOK()) rpm = 0;
        io.setShooterVelocity(rpm);
    }

    private void setFeederVelocity(double rpm) {
        if (!hardwareOK()) rpm = 0;
        io.setFeederVelocity(rpm);
    }

    private void setShooterWithSubshooter(double baseRPM) {
        if (!hardwareOK()) baseRPM = 0;
        io.setShooterWithSubshooter(baseRPM);
    }

    private void executeIdle() {
        setShooterMotorVolts(0.0);
        setFeederMotorVolts(0.0);
    }

    public void updateAutoSetpoint(Pose2d robotPose) {
        var result = ShooterInterpolation.calculate(robotPose);
        autoShooterRPM = result.shooterRPM();
        distToHub = result.distance();
    }

    public double getAutoShooterRPM() {
        return autoShooterRPM;
    }

    public Command runShooter(double shooterMotorVolts) {
        return run(() -> setShooterMotorVolts(shooterMotorVolts));
    }

    public Command runFeeder(double feederMotorVolts) {
        return run(() -> setFeederMotorVolts(feederMotorVolts));
    }

    public Command runShooterVelocity(double rpm) {
        return run(() -> setShooterVelocity(rpm));
    }

    public Command runShooterVelocity(DoubleSupplier rpmSupplier) {
        {
            return run(() -> setShooterVelocity(rpmSupplier.getAsDouble()));
        }
    }

    public Command runFeederVelocity(double rpm) {
        return run(() -> setFeederVelocity(rpm));
    }

    public Command runSetPoint(double shooterRPM, double feederRPM) {
        return runShooterVelocity(shooterRPM).alongWith(runFeederVelocity(feederRPM));
    }

    public Command runIdle() {
        return runShooter(0).alongWith(runFeeder(0));
    }

    /**
     * Shooter control with differential speed shooter set to baseRPM, subshooter set to baseRPM + SUBSHOOTER_RPM_OFFSET
     *
     * @param baseRPM base speed (shooter speed)
     * @return Command
     */
    public Command runShooterWithSubshooter(double baseRPM) {
        return run(() -> setShooterWithSubshooter(baseRPM));
    }

    /**
     * Shooter control with differential speed (dynamic RPM)
     *
     * @param baseRPMSupplier base speed supplier
     * @return Command
     */
    public Command runShooterWithSubshooter(DoubleSupplier baseRPMSupplier) {
        return run(() -> setShooterWithSubshooter(baseRPMSupplier.getAsDouble()));
    }

    /** Runs shooter/subshooter and feeder together from one command execute loop. */
    public Command runShooterAndFeeder(DoubleSupplier shooterRpmSupplier, double feederRpm) {
        return run(() -> {
            setShooterWithSubshooter(shooterRpmSupplier.getAsDouble());
            setFeederVelocity(feederRpm);
        });
    }

    /**
     * Hold-to-shoot command: keep shooter spinning, and only enable feeder after shooter reaches speed.
     *
     * @param shooterRpmSupplier shooter target RPM supplier
     * @param feederRpm feeder RPM once shooter is ready
     * @param shooterReadyToleranceRpm shooter ready tolerance (RPM)
     * @return command requiring only this subsystem
     */
    public Command runShooterThenFeeder(
            DoubleSupplier shooterRpmSupplier, double feederRpm, double shooterReadyToleranceRpm) {
        final boolean[] feederLatchedOn = new boolean[] {false};

        return run(() -> {
                    double shooterTargetRpm = shooterRpmSupplier.getAsDouble();
                    setShooterWithSubshooter(shooterTargetRpm);

                    // Latch feeder once shooter first reaches speed to avoid rapid on/off toggling
                    // caused by transient RPM dips when game piece enters the shooter.
                    if (isShooterAtSpeed(shooterTargetRpm, shooterReadyToleranceRpm)) {
                        feederLatchedOn[0] = true;
                    }

                    if (feederLatchedOn[0]) {
                        setFeederVelocity(feederRpm);
                    } else {
                        setFeederVelocity(0.0);
                    }

                    Logger.recordOutput("Shooter/FeederLatchedOn", feederLatchedOn[0]);
                })
                .beforeStarting(() -> feederLatchedOn[0] = false);
    }

    /** Stops shooter, subshooter, and feeder together in one command. */
    public Command stopAllShooterMotors() {
        return run(() -> {
            setShooterWithSubshooter(0.0);
            setFeederVelocity(0.0);
        });
    }

    public Command runAutoShooterWithFeeder(double feederRpm) {
        return run(() -> {
            io.setShooterWithSubshooter(autoShooterRPM);

            if (isShooterAtSpeed(autoShooterRPM, ShooterContants.SHOOTER_READY_TOLERANCE_RPM)) {
                setFeederVelocity(feederRpm);
            } else {
                setFeederVelocity(0.0);
            }
        });
    }

    public Command runAutoRPMCommand(Supplier<Pose2d> robotPoseSupplier, Translation2d targetLocation) {
        return run(() -> {
            Pose2d currentPose = robotPoseSupplier.get();

            double distance = currentPose.getTranslation().getDistance(targetLocation);

            double targetRpm = ShooterInterpolation.getRpm(distance);

            if (isShooterAtSpeed(autoShooterRPM, ShooterContants.SHOOTER_READY_TOLERANCE_RPM)) {
                setFeederVelocity(targetRpm);
            } else {
                setFeederVelocity(0.0);
            }

            Logger.recordOutput("Shooter/AutoDistance", distance);
            Logger.recordOutput("Shooter/AutoTargetRPM", targetRpm);
        });
    }

    /**
     * Checks whether the shooter wheel average speed has reached the target speed.
     *
     * <p>Uses absolute value comparison so target sign conventions do not affect readiness check.
     */
    public boolean isShooterAtSpeed(double targetRpm, double toleranceRpm) {
        if (!hardwareOK()) return false;
        if (inputs.shooterMotorsVelocityRPM == null || inputs.shooterMotorsVelocityRPM.length == 0) return false;

        // Use average magnitude to avoid sign cancellation when some motors are configured as opposed followers.
        double averageShooterRpm = calculateAverageAbsoluteVelocity(inputs.shooterMotorsVelocityRPM);
        boolean ready = Math.abs(averageShooterRpm - Math.abs(targetRpm)) <= Math.abs(toleranceRpm);

        Logger.recordOutput("Shooter/ReadyCheck/TargetRPM", Math.abs(targetRpm));
        Logger.recordOutput("Shooter/ReadyCheck/AverageRPM", averageShooterRpm);
        Logger.recordOutput("Shooter/ReadyCheck/ToleranceRPM", Math.abs(toleranceRpm));
        Logger.recordOutput("Shooter/ReadyCheck/IsReady", ready);

        return ready;
    }
}
