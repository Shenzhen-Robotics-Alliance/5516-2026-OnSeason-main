// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autos.*;
import frc.robot.commands.drive.*;
import frc.robot.constants.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.IO.*;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.subsystems.vision.apriltags.AprilTagVisionIOReal;
import frc.robot.subsystems.vision.apriltags.ApriltagVisionIOSim;
import frc.robot.subsystems.vision.apriltags.PhotonCameraProperties;
import frc.robot.utils.AlertsManager;
import frc.robot.utils.MapleJoystickDriveInput;
import java.util.*;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;
import org.ironmaple.utils.mathutils.MapleCommonMath;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static final boolean SIMULATE_AUTO_PLACEMENT_INACCURACY = true;

    // pdp for akit logging
    public final LoggedPowerDistribution powerDistributionLog;
    // Subsystems
    public final SwerveDrive drive;
    public final AprilTagVision aprilTagVision;
    public final LEDStatusLight ledStatusLight;

    // Controller
    // public final DriverMap driver = new DriverMap.LeftHandedPS5(0);
    public final DriverMap driver = new DriverMap.LeftHandedXbox(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    private final LoggedDashboardChooser<Auto> autoChooser;
    private final SendableChooser<Supplier<Command>> testChooser;

    // Simulated drive
    private final SwerveDriveSimulation driveSimulation;
    // public final CoralHolder coralHolder;
    // private final Climb climb;

    private final Field2d field = new Field2d();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        final List<PhotonCameraProperties> camerasProperties =
                VisionConstants.photonVisionCameras; // load configs stored directly in VisionConstants.java

        switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                driveSimulation = null;

                powerDistributionLog = LoggedPowerDistribution.getInstance(1, PowerDistribution.ModuleType.kRev);

                /* CTRE Chassis: */
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.CTRE_TIME_SYNCHRONIZED,
                        new GyroIOPigeon2(TunerConstants.DrivetrainConstants, false),
                        new CanBusIOReal(TunerConstants.kCANBus),
                        new ModuleIOTalon(TunerConstants.FrontLeft, "FrontLeft"),
                        new ModuleIOTalon(TunerConstants.FrontRight, "FrontRight"),
                        new ModuleIOTalon(TunerConstants.BackLeft, "BackLeft"),
                        new ModuleIOTalon(TunerConstants.BackRight, "BackRight"));

                aprilTagVision = new AprilTagVision(new AprilTagVisionIOReal(camerasProperties), camerasProperties);
            }

            case SIM -> {
                SimulatedArena.overrideSimulationTimings(
                        Seconds.of(Robot.defaultPeriodSecs), DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD);
                this.driveSimulation = new SwerveDriveSimulation(
                        DriveTrainSimulationConfig.Default()
                                .withRobotMass(DriveTrainConstants.ROBOT_MASS)
                                .withBumperSize(DriveTrainConstants.BUMPER_LENGTH, DriveTrainConstants.BUMPER_WIDTH)
                                .withTrackLengthTrackWidth(
                                        DriveTrainConstants.TRACK_LENGTH, DriveTrainConstants.TRACK_WIDTH)
                                .withSwerveModule(new SwerveModuleSimulationConfig(
                                        DriveTrainConstants.DRIVE_MOTOR_MODEL,
                                        DriveTrainConstants.STEER_MOTOR_MODEL,
                                        DriveTrainConstants.DRIVE_GEAR_RATIO,
                                        DriveTrainConstants.STEER_GEAR_RATIO,
                                        DriveTrainConstants.DRIVE_FRICTION_VOLTAGE,
                                        DriveTrainConstants.STEER_FRICTION_VOLTAGE,
                                        DriveTrainConstants.WHEEL_RADIUS,
                                        DriveTrainConstants.STEER_INERTIA,
                                        DriveTrainConstants.WHEEL_COEFFICIENT_OF_FRICTION))
                                .withGyro(DriveTrainConstants.gyroSimulationFactory),
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                powerDistributionLog = LoggedPowerDistribution.getInstance();

                // Sim robot, instantiate physics sim IO implementations
                final ModuleIOSim frontLeft = new ModuleIOSim(driveSimulation.getModules()[0]),
                        frontRight = new ModuleIOSim(driveSimulation.getModules()[1]),
                        backLeft = new ModuleIOSim(driveSimulation.getModules()[2]),
                        backRight = new ModuleIOSim(driveSimulation.getModules()[3]);
                final GyroIOSim gyroIOSim = new GyroIOSim(driveSimulation.getGyroSimulation());
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.GENERIC,
                        gyroIOSim,
                        (canBusInputs) -> {},
                        frontLeft,
                        frontRight,
                        backLeft,
                        backRight);

                aprilTagVision = new AprilTagVision(
                        new ApriltagVisionIOSim(
                                camerasProperties,
                                VisionConstants.fieldLayout,
                                driveSimulation::getSimulatedDriveTrainPose),
                        camerasProperties);

                SimulatedArena.getInstance().resetFieldForAuto();
            }

            default -> {
                this.driveSimulation = null;

                powerDistributionLog = LoggedPowerDistribution.getInstance();

                // Replayed robot, disable IO implementations
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.GENERIC,
                        (canBusInputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {});

                aprilTagVision = new AprilTagVision((inputs) -> {}, camerasProperties);
            }
        }

        this.ledStatusLight = new LEDStatusLight(0, 26, false);

        this.drive.configHolonomicPathPlannerAutoBuilder(field);

        SmartDashboard.putData("Select Test", testChooser = buildTestsChooser());
        autoChooser = buildAutoChooser();

        configureButtonBindings();

        SmartDashboard.putData("Field", field);

        setMotorBrake(true);
    }

    private void configureAutoTriggers(PathPlannerAuto pathPlannerAuto) {}

    private LoggedDashboardChooser<Auto> buildAutoChooser() {
        final LoggedDashboardChooser<Auto> autoSendableChooser = new LoggedDashboardChooser<>("Select Auto");
        autoSendableChooser.addDefaultOption("None", Auto.none());

        SmartDashboard.putData("Select Auto", autoSendableChooser.getSendableChooser());
        return autoSendableChooser;
    }

    private SendableChooser<Supplier<Command>> buildTestsChooser() {
        final SendableChooser<Supplier<Command>> testsChooser = new SendableChooser<>();
        testsChooser.setDefaultOption("None", Commands::none);
        testsChooser.addOption(
                "Drive SysId- Quasistatic - Forward", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        testsChooser.addOption(
                "Drive SysId- Quasistatic - Reverse", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        testsChooser.addOption(
                "Drive SysId- Dynamic - Forward", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        testsChooser.addOption(
                "Drive SysId- Dynamic - Reverse", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        return testsChooser;
    }

    private boolean isDSPresentedAsRed = FieldMirroringUtils.isSidePresentedAsRed();
    private Command autonomousCommand = Commands.none();
    private Auto previouslySelectedAuto = Auto.none();
    /** reconfigures button bindings if alliance station has changed re-create autos if not yet created */
    public void checkForCommandChanges() {
        final Auto selectedAuto = autoChooser.get();
        if (FieldMirroringUtils.isSidePresentedAsRed() == isDSPresentedAsRed & selectedAuto == previouslySelectedAuto)
            return;

        try {
            this.autonomousCommand = selectedAuto.getAutoCommand(this);
            configureAutoTriggers(new PathPlannerAuto(autonomousCommand, selectedAuto.getStartingPoseAtBlueAlliance()));
            System.out.println("Auto updated to: " + selectedAuto.getClass());
        } catch (Exception e) {
            this.autonomousCommand = Commands.none();
            DriverStation.reportError(
                    "Error Occurred while obtaining autonomous command: \n"
                            + e.getMessage()
                            + "\n"
                            + Arrays.toString(e.getStackTrace()),
                    false);
            throw new RuntimeException(e);
        }
        resetFieldAndOdometryForAuto(selectedAuto.getStartingPoseAtBlueAlliance());

        previouslySelectedAuto = selectedAuto;
        isDSPresentedAsRed = FieldMirroringUtils.isSidePresentedAsRed();
    }

    private void resetFieldAndOdometryForAuto(Pose2d robotStartingPoseAtBlueAlliance) {
        final Pose2d startingPose = FieldMirroringUtils.toCurrentAlliancePose(robotStartingPoseAtBlueAlliance);

        if (driveSimulation != null) {
            Transform2d placementError = SIMULATE_AUTO_PLACEMENT_INACCURACY
                    ? new Transform2d(
                            MapleCommonMath.generateRandomNormal(0, 0.2),
                            MapleCommonMath.generateRandomNormal(0, 0.2),
                            Rotation2d.fromDegrees(MapleCommonMath.generateRandomNormal(0, 1)))
                    : new Transform2d();
            driveSimulation.setSimulationWorldPose(startingPose.plus(placementError));
            SimulatedArena.getInstance().resetFieldForAuto();
        }

        aprilTagVision
                .focusOnTarget(-1, -1)
                .withTimeout(0.1)
                .alongWith(Commands.runOnce(() -> drive.setPose(startingPose), drive))
                .ignoringDisable(true)
                .schedule();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link XboxController}), and then passing it to
     * a {@link JoystickButton}.
     */
    public void configureButtonBindings() {
        SmartDashboard.putData(
                "Enable Motor Brake",
                Commands.runOnce(() -> setMotorBrake(true))
                        .onlyIf(DriverStation::isDisabled)
                        .beforeStarting(Commands.print("unlocking motor brakes..."))
                        .andThen(Commands.print("motor brakes unlocked!"))
                        .ignoringDisable(true));
        SmartDashboard.putData(
                "Disable Motor Brake",
                Commands.runOnce(() -> setMotorBrake(false))
                        .onlyIf(DriverStation::isDisabled)
                        .beforeStarting(Commands.print("locking motor brakes..."))
                        .andThen(Commands.print("motor brakes locked!"))
                        .ignoringDisable(true));

        /* joystick drive command */
        final MapleJoystickDriveInput driveInput = driver.getDriveInput();
        IntSupplier pov =
                // driver.getController().getHID()::getPOV;
                () -> -1;
        final JoystickDrive joystickDrive = new JoystickDrive(driveInput, () -> true, pov, drive);
        drive.setDefaultCommand(joystickDrive.ignoringDisable(true));
        JoystickDrive.instance = Optional.of(joystickDrive);

        /* reset gyro heading manually (in case the vision does not work) */
        driver.resetOdometryButton()
                .onTrue(Commands.runOnce(
                                () -> drive.setPose(new Pose2d(
                                        drive.getPose().getTranslation(),
                                        FieldMirroringUtils.getCurrentAllianceDriverStationFacing())),
                                drive)
                        .ignoringDisable(true));

        /* lock chassis with x-formation */
        driver.lockChassisWithXFormatButton().whileTrue(drive.lockChassisWithXFormation());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonomousCommand;
    }

    public Command getTestCommand() {
        return testChooser.getSelected().get();
    }

    public void updateFieldSimAndDisplay() {
        if (driveSimulation == null) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    }

    private final Alert autoPlacementIncorrect = AlertsManager.create(
            "Expected Autonomous robot placement position does not match reality, IS THE SELECTED AUTO CORRECT?",
            Alert.AlertType.kWarning);
    private final Alert lowBattery =
            AlertsManager.create("Battery voltage 12.0, please keep it above 12.5V", Alert.AlertType.kInfo);
    private static final double AUTO_PLACEMENT_TOLERANCE_METERS = 0.25;
    private static final double AUTO_PLACEMENT_TOLERANCE_DEGREES = 5;

    public void updateTelemetryAndLED() {
        field.setRobotPose(
                Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
                        ? driveSimulation.getSimulatedDriveTrainPose()
                        : drive.getPose());
        if (Robot.CURRENT_ROBOT_MODE == RobotMode.SIM)
            field.getObject("Odometry").setPose(drive.getPose());

        Pose2d autoStartingPose =
                FieldMirroringUtils.toCurrentAlliancePose(previouslySelectedAuto.getStartingPoseAtBlueAlliance());
        Pose2d currentPose = RobotState.getInstance().getVisionPose();
        Transform2d difference = autoStartingPose.minus(currentPose);
        boolean autoPlacementIncorrectDetected = difference.getTranslation().getNorm() > AUTO_PLACEMENT_TOLERANCE_METERS
                || Math.abs(difference.getRotation().getDegrees()) > AUTO_PLACEMENT_TOLERANCE_DEGREES;
        // autoPlacementIncorrect.set(autoPlacementIncorrectDetected && DriverStation.isDisabled());
        autoPlacementIncorrect.set(false);

        double voltage = ConduitApi.getInstance().getPDPVoltage();
        lowBattery.setText(String.format(
                "Battery voltage: %.1fV, please try to keep it above 12.5V before going on field", voltage));
        lowBattery.set(DriverStation.isDisabled() && voltage < 12.5);

        AlertsManager.updateLEDAndLog(ledStatusLight);
    }

    public static boolean motorBrakeEnabled = false;

    public void setMotorBrake(boolean brakeModeEnabled) {
        if (motorBrakeEnabled == brakeModeEnabled) return;

        System.out.println("Set motor brake: " + brakeModeEnabled);
        drive.setMotorBrake(brakeModeEnabled);

        motorBrakeEnabled = brakeModeEnabled;
    }
}
