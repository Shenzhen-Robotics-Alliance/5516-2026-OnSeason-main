// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterContants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.drive.HubAlignmentCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.vision.*;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private static final double AUTO_SHOOT_TIMEOUT_SECS = 2.0;
    
    /** Climb alignment tags that can trigger vision calibration mode. */
    private static final int[] CLIMB_ALIGNMENT_TAG_WHITELIST = new int[] {15, 16, 31, 32};

    private enum SotfAimMode {
        SOTF_AUTO,
        MANUAL
    }

    // Subsystems
    private final Vision vision;
    private final Drive drive;
    private SwerveDriveSimulation driveSimulation = null;

    public Shooter shooter;
    public Arm arm;
    public Climb climb;

    // Controller
    // private final CommandXboxController controller = new CommandXboxController(0);
    public final DriverMap controller = new DriverMap.LeftHandedXbox(0);
    /** Copilot controller, only used for entering climb vision calibration with X button. */
    public final CommandXboxController copilotController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private SotfAimMode currentSotfAimMode = SotfAimMode.SOTF_AUTO;
    /** Latches true after copilot presses X to arm climb vision calibration mode. */
    private boolean climbVisionCalibrationArmed = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        (robotPose) -> {});

                vision = new Vision(
                        drive,
                        new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                        new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1));
                arm = new Arm(new ArmIOReal());
                climb = new Climb(new ClimbIOReal());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation =
                        new SwerveDriveSimulation(Drive.getMapleSimConfig(), new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                climb = new Climb(new ClimbIO() {
                    @Override
                    public void updateInputs(ClimbInputs inputs) {}
                });
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (robotPose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                climb = new Climb(new ClimbIO() {
                    @Override
                    public void updateInputs(ClimbInputs inputs) {}
                });
                break;
        }

        shooter = new Shooter(new ShooterIOReal());
        arm = new Arm(new ArmIOReal());

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        // drive.setDefaultCommand(DriveCommands.joystickDrive(
        //         drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        SmartDashboard.putData(
                "Enable Motor Brake",
                Commands.runOnce(() -> setMotorBrake(true)).ignoringDisable(true));
        SmartDashboard.putData(
                "Disable Motor Brake",
                Commands.runOnce(() -> setMotorBrake(false)).ignoringDisable(true));

        // Keep one single driving workflow for the driver in both modes.
        // Auto-aim only adds heading assistance and does not replace the base control scheme.
        drive.setDefaultCommand(DriveCommands.joystickDriveWithOptionalAutoAim(
                drive,
                () -> controller.translationalAxisY().getAsDouble(),
                () -> controller.translationalAxisX().getAsDouble(),
                () -> -controller.rotationalAxisX().getAsDouble(),
                this::shouldRunTeleopAutoAim,
                this::getAutoAimTargetRotation));

        // Operator mode selection:
        // A => enable SOTF auto-aim mode
        // B => force pure manual aiming mode
        controller.enableSotfAutoAimButton().onTrue(Commands.runOnce(() -> currentSotfAimMode = SotfAimMode.SOTF_AUTO));
        controller.enableManualAimButton().onTrue(Commands.runOnce(() -> currentSotfAimMode = SotfAimMode.MANUAL));
        // Copilot X: enter climb vision calibration mode.
        // Calibration will only activate when one of tags {15, 16, 31, 32} becomes visible.
        copilotController.x().onTrue(Commands.runOnce(() -> climbVisionCalibrationArmed = true));

        // Lock to 0° when A button is held
        // controller
        //         .a()
        //         .whileTrue(DriveCommands.joystickDriveAtAngle(
        //                 drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> new Rotation2d()));

        // controller
        //         .lockToZeroAngle()
        //         .whileTrue(DriveCommands.joystickDriveAtAngle(
        //                 drive,
        //                 () -> controller.translationalAxisY().getAsDouble(),
        //                 () -> controller.translationalAxisX().getAsDouble(),
        //                 () -> new Rotation2d(vision.getTargetX(1).getDegrees())));

        // Switch to X pattern when X button is pressed
        // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        controller.lockChassisWithXFormatButton().whileTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
        final Runnable resetOdometry = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
                : () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
        // controller.start().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));
        controller.resetOdometryButton().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));
        SmartDashboard.putNumber("Shooter Velocity (RPM)", -2800.0);
        DoubleSupplier shooterVelocitySupplier = () -> SmartDashboard.getNumber("Shooter Velocity (RPM)", -2800.0);
        controller
                // Press left bumper once to toggle arm between start and intake positions.
                .startShooterMotorButton()
                .onTrue(arm.toggleArmPositionCommand());

        controller
                .startFeederToShootButton()
                // Hold right trigger: spin up shooter first, then start feeder after shooter reaches target speed.
                .whileTrue(shooter.runShooterThenFeeder(
                        shooterVelocitySupplier, FEEDER_SHOOT_RPM, SHOOTER_READY_TOLERANCE_RPM))
                // Release right trigger: stop both shooter and feeder immediately.
                .onFalse(shooter.stopAllShooterMotors());

        Trigger outtakeEnabledTrigger = controller.spitOutButton().and(new Trigger(arm::isAtIntakePosition));
        outtakeEnabledTrigger
                // Hold back: reverse feeder and intake to eject game pieces only at intake position.
                .whileTrue(shooter.runFeederVelocity(FEEDER_OUTTAKE_RPM).alongWith(arm.outtakeCommand()));
        controller
                .spitOutButton()
                // Always stop eject motors when back is released.
                .onFalse(shooter.runFeederVelocity(0.0).alongWith(arm.intakeIdleCommand()));
        // Auto-aiming binding with vision (uses AprilTag detection)
        controller
                .autoAlignToHubButton()
                .whileTrue(HubAlignmentCommands.aimAtHubWithVision(
                        drive,
                        vision,
                        () -> controller.translationalAxisY().getAsDouble(),
                        () -> controller.translationalAxisX().getAsDouble(),
                        DriveCommands.BLUE_TARGET_POSITION));

        controller
                // Hold left trigger: run intake only. Release to stop intake.
                .intakeButton()
                .whileTrue(arm.intakeCommand())
                .onFalse(arm.intakeIdleCommand());

        controller
                // Hold POV up: climb moves upward, release to stop.
                .povUp()
                .whileTrue(climb.manualUpCommand());
        controller
                // Hold POV down: climb moves downward, release to stop.
                .povDown()
                .whileTrue(climb.manualDownCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    private boolean motorBrakeEnabled = false;

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
    }

    private Translation2d getAllianceFallbackTarget() {
        if (isRedAlliance()) {
            return new Translation2d(
                    16.54 - DriveCommands.BLUE_TARGET_POSITION.getX(), DriveCommands.BLUE_TARGET_POSITION.getY());
        }
        return DriveCommands.BLUE_TARGET_POSITION;
    }

    /**
     * Returns the heading target for teleop auto-aim.
     *
     * <p>If climb vision calibration is active, face the nearest visible climb alignment tag. Otherwise use existing
     * SOTF hub target logic.
     */
    private Rotation2d getAutoAimTargetRotation() {
        Translation2d targetPosition = isClimbVisionCalibrationActive()
                ? getVisibleClimbAlignmentTagPosition().orElseGet(this::getAllianceFallbackTarget)
                : vision.getVisibleHubPosition(isRedAlliance()).orElseGet(this::getAllianceFallbackTarget);
        Translation2d robotPosition = drive.getPose().getTranslation();
        Translation2d delta = targetPosition.minus(robotPosition);
        return new Rotation2d(Math.atan2(delta.getY(), delta.getX()));
    }

    /**
     * Finds the nearest visible climb alignment AprilTag position from the whitelist.
     *
     * @return Optional nearest tag position on field if visible
     */
    private java.util.Optional<Translation2d> getVisibleClimbAlignmentTagPosition() {
        Translation2d robotPosition = drive.getPose().getTranslation();
        Translation2d bestPosition = null;
        double bestDistanceMeters = Double.POSITIVE_INFINITY;

        for (int tagId : CLIMB_ALIGNMENT_TAG_WHITELIST) {
            var tagPose = vision.getTagPose(tagId);
            if (tagPose.isEmpty()) {
                continue;
            }

            Translation2d tagPosition = tagPose.get().toPose2d().getTranslation();
            double distanceMeters = tagPosition.getDistance(robotPosition);
            if (distanceMeters < bestDistanceMeters) {
                bestDistanceMeters = distanceMeters;
                bestPosition = tagPosition;
            }
        }

        return bestPosition == null ? java.util.Optional.empty() : java.util.Optional.of(bestPosition);
    }

    /** Returns true when copilot has armed climb vision calibration and a climb alignment tag is currently visible. */
    private boolean isClimbVisionCalibrationActive() {
        return climbVisionCalibrationArmed
                && getVisibleClimbAlignmentTagPosition().isPresent();
    }

    /**
     * Unified teleop auto-aim gate.
     *
     * <p>Behavior:
     *
     * <p>1) When climb vision calibration is armed, only climb-tag calibration can activate auto-aim.
     *
     * <p>2) Otherwise preserve the original SOTF auto-aim behavior.
     */
    private boolean shouldRunTeleopAutoAim() {
        boolean climbTagVisible = getVisibleClimbAlignmentTagPosition().isPresent();
        boolean climbCalibrationActive = climbVisionCalibrationArmed && climbTagVisible;

        boolean sotfAutoModeEnabled = currentSotfAimMode == SotfAimMode.SOTF_AUTO;
        boolean sotfHasWhitelistedTag = vision.hasSotfTarget();
        boolean sotfAutoAimActive = !climbVisionCalibrationArmed && sotfAutoModeEnabled && sotfHasWhitelistedTag;

        Logger.recordOutput("SOTF/Mode", currentSotfAimMode.toString());
        Logger.recordOutput("SOTF/AutoModeEnabled", sotfAutoModeEnabled);
        Logger.recordOutput("SOTF/HasWhitelistedTag", sotfHasWhitelistedTag);
        Logger.recordOutput("SOTF/AutoAimActive", sotfAutoAimActive);
        Logger.recordOutput("ClimbVision/Armed", climbVisionCalibrationArmed);
        Logger.recordOutput("ClimbVision/TagVisible", climbTagVisible);
        Logger.recordOutput("ClimbVision/AutoCalibrationActive", climbCalibrationActive);
        Logger.recordOutput("ClimbVision/WhitelistedTagIds", CLIMB_ALIGNMENT_TAG_WHITELIST);

        return climbCalibrationActive || sotfAutoAimActive;
    }

    public void setMotorBrake(boolean brakeModeEnable) {
        if (this.motorBrakeEnabled == brakeModeEnable) return;
        System.out.println("Set motor brake: " + brakeModeEnable);
        drive.setMotorBrake(brakeModeEnable);
        arm.setIntakeMotorBrake(brakeModeEnable);
        arm.setArmMotorBrake(brakeModeEnable);

        this.motorBrakeEnabled = brakeModeEnable;
    }
}
