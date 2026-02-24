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
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOReal;
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
    // Subsystems
    private final Vision vision;
    private final Drive drive;
    private SwerveDriveSimulation driveSimulation = null;

    public Shooter shooter;
    public Arm arm;

    // Controller
    //     private final CommandXboxController controller = new CommandXboxController(0);
    public final DriverMap controller = new DriverMap.RightHandedXbox(0);
    public final CommandXboxController viceController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

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
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
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

        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> controller.translationalAxisY().getAsDouble(),
                () -> controller.translationalAxisX().getAsDouble(),
                () -> -controller.rotationalAxisX().getAsDouble()));

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
        SmartDashboard.putNumber("Shooter Velocity (RPM)", -3000.0);
        DoubleSupplier shooterVelocitySupplier = () -> SmartDashboard.getNumber("Shooter Velocity (RPM)", -3000.0);
        controller
                .startShooterMotorButton()
                .onTrue(shooter.runShooterVelocity(shooterVelocitySupplier))
                .onFalse(shooter.runShooterVelocity(0.0));

        controller
                .startFeederToShootButton()
                .whileTrue(shooter.runFeederVelocity(2000).alongWith(arm.intakeCommand()))
                .whileFalse(shooter.runFeederVelocity(0.0).alongWith(arm.intakeIdleCommand()));
        // Auto-aiming binding
        controller
                .autoAlignToHubButton()
                .whileTrue(DriveCommands.autoAim(
                        drive,
                        () -> controller.translationalAxisY().getAsDouble(),
                        () -> controller.translationalAxisX().getAsDouble()));

        controller.intakeButton().whileTrue(arm.intakeCommand());

        viceController.leftBumper().onTrue(arm.armDroppingCommand());
        viceController.rightBumper().onTrue(arm.armUprightCommand());
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

    public void setMotorBrake(boolean brakeModeEnable) {
        if (this.motorBrakeEnabled == brakeModeEnable) return;
        System.out.println("Set motor brake: " + brakeModeEnable);
        drive.setMotorBrake(brakeModeEnable);
        arm.setIntakeMotorBrake(brakeModeEnable);
        arm.setArmMotorBrake(brakeModeEnable);

        this.motorBrakeEnabled = brakeModeEnable;
    }
}
