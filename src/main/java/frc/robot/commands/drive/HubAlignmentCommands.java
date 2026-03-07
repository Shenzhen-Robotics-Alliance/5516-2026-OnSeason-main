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

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Commands for aligning the robot to the hub with velocity prediction.
 *
 * <p>This provides feedforward + feedback control for more accurate aiming when the robot is moving.
 */
public class HubAlignmentCommands {
    // PID constants for angle control
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.2;
    private static final double ANGLE_MAX_VELOCITY = 12.0;
    private static final double ANGLE_MAX_ACCELERATION = 30.0;

    // Feedforward gain for velocity prediction (0.7 = 70% feedforward)
    public static final double FEED_FORWARD_RATE = 0.7;

    // Robot control period (20ms)
    private static final double ROBOT_PERIOD_SECS = 0.02;

    // Field dimensions for alliance mirroring (2026 FRC field)
    private static final double FIELD_LENGTH_METERS = 16.54;
    private static final double FIELD_WIDTH_METERS = 8.02;

    private HubAlignmentCommands() {}

    /**
     * Mirrors a target position for red alliance.
     *
     * @param blueTarget Target position for blue alliance
     * @return Mirrored position for red alliance, or original if blue
     */
    private static Translation2d mirrorForAlliance(Translation2d blueTarget) {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red) {
            // Mirror across field center
            return new Translation2d(FIELD_LENGTH_METERS - blueTarget.getX(), blueTarget.getY());
        }
        return blueTarget;
    }

    /**
     * Creates a command that drives while aiming at a target with velocity prediction.
     *
     * <p>This uses feedforward + feedback control to predict where the robot will be and aim accordingly, providing
     * more accurate aiming when moving.
     *
     * @param drive The drive subsystem
     * @param xSupplier Joystick X input (forward/backward)
     * @param ySupplier Joystick Y input (left/right)
     * @param targetSupplier Supplier for the target position on the field
     * @return The command that drives while aiming at the target
     */
    public static Command driveAndAimAtTarget(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Translation2d> targetSupplier) {
        return new DriveAndAimAtTargetCommand(drive, xSupplier, ySupplier, targetSupplier);
    }

    /**
     * Creates a command that aims at a target with alliance-aware target selection.
     *
     * <p>Automatically mirrors the target position for red alliance.
     *
     * @param drive The drive subsystem
     * @param xSupplier Joystick X input
     * @param ySupplier Joystick Y input
     * @param blueAllianceTarget Target position for blue alliance
     * @return The command that drives while aiming at the target
     */
    public static Command aimAtTargetWithAlliance(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Translation2d blueAllianceTarget) {
        return new AimAtTargetWithAllianceCommand(drive, xSupplier, ySupplier, blueAllianceTarget);
    }

    /**
     * Creates a command that aims at the hub using vision AprilTag detection.
     *
     * <p>Uses the vision system to find the nearest visible hub AprilTag and aims at it. This provides dynamic,
     * real-time hub tracking without hardcoded positions.
     *
     * @param drive The drive subsystem
     * @param vision The vision subsystem
     * @param xSupplier Joystick X input
     * @param ySupplier Joystick Y input
     * @param fallbackTarget Fallback target position if no hub tag is visible
     * @return The command that drives while aiming at the hub
     */
    public static Command aimAtHubWithVision(
            Drive drive,
            Vision vision,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Translation2d fallbackTarget) {
        return new AimAtHubWithVisionCommand(drive, vision, xSupplier, ySupplier, fallbackTarget);
    }

    /**
     * Command that drives while aiming at a target with velocity prediction.
     *
     * <p>This uses feedforward + feedback control to predict where the robot will be and aim accordingly.
     */
    private static class DriveAndAimAtTargetCommand extends Command {
        private final Drive drive;
        private final DoubleSupplier xSupplier;
        private final DoubleSupplier ySupplier;
        private final Supplier<Translation2d> targetSupplier;
        private final ProfiledPIDController angleController;

        DriveAndAimAtTargetCommand(
                Drive drive,
                DoubleSupplier xSupplier,
                DoubleSupplier ySupplier,
                Supplier<Translation2d> targetSupplier) {
            this.drive = drive;
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.targetSupplier = targetSupplier;
            this.angleController = new ProfiledPIDController(
                    ANGLE_KP,
                    0.0,
                    ANGLE_KD,
                    new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
            angleController.enableContinuousInput(-Math.PI, Math.PI);

            addRequirements(drive);
        }

        @Override
        public void initialize() {
            angleController.reset(drive.getRotation().getRadians());
        }

        @Override
        public void execute() {
            // Get linear velocity from joysticks (simplified - no deadband for alignment)
            double x = xSupplier.getAsDouble();
            double y = ySupplier.getAsDouble();
            Translation2d linearVelocity = new Translation2d(x, y).times(drive.getMaxLinearSpeedMetersPerSec());

            // Get robot position and velocity
            Translation2d robotPosition = drive.getPose().getTranslation();
            ChassisSpeeds robotVelocity = drive.getChassisSpeeds();

            // Predict robot position after one control cycle
            Translation2d predictedRobotPosition = robotPosition.plus(new Translation2d(
                    robotVelocity.vxMetersPerSecond * ROBOT_PERIOD_SECS,
                    robotVelocity.vyMetersPerSecond * ROBOT_PERIOD_SECS));

            // Calculate target angle considering velocity prediction
            Translation2d targetPosition = targetSupplier.get();
            Translation2d delta = targetPosition.minus(predictedRobotPosition);
            Rotation2d targetFacing = new Rotation2d(Math.atan2(delta.getY(), delta.getX()));

            // Calculate angle change rate for feedforward
            Translation2d currentDelta = targetPosition.minus(robotPosition);
            Rotation2d currentFacing = new Rotation2d(Math.atan2(currentDelta.getY(), currentDelta.getX()));
            double angleChangeRate = targetFacing.minus(currentFacing).getRadians() / ROBOT_PERIOD_SECS;

            // Calculate feedback (PID)
            double feedback = angleController.calculate(drive.getRotation().getRadians(), targetFacing.getRadians());

            // Calculate feedforward
            double feedforward = angleChangeRate * FEED_FORWARD_RATE;

            // Combine feedforward and feedback
            double omega = feedback + feedforward;

            // Log data for AdvantageScope
            Logger.recordOutput("HubAlignment/TargetAngle", targetFacing.getDegrees());
            Logger.recordOutput("HubAlignment/CurrentAngle", drive.getRotation().getDegrees());
            Logger.recordOutput("HubAlignment/Feedback", feedback);
            Logger.recordOutput("HubAlignment/Feedforward", feedforward);
            Logger.recordOutput("HubAlignment/Omega", omega);
            Logger.recordOutput("HubAlignment/PredictedPositionX", predictedRobotPosition.getX());
            Logger.recordOutput("HubAlignment/PredictedPositionY", predictedRobotPosition.getY());
            Logger.recordOutput("HubAlignment/LinearVelocityX", linearVelocity.getX());
            Logger.recordOutput("HubAlignment/LinearVelocityY", linearVelocity.getY());

            // Convert to field-relative speeds
            ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

            boolean isFlipped = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds, isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
        }

        @Override
        public void end(boolean interrupted) {
            drive.stop();
        }
    }

    /**
     * Command that aims at a target with alliance-aware target selection.
     *
     * <p>Automatically mirrors the target position for red alliance.
     */
    private static class AimAtTargetWithAllianceCommand extends Command {
        private final Drive drive;
        private final DoubleSupplier xSupplier;
        private final DoubleSupplier ySupplier;
        private final Translation2d blueAllianceTarget;
        private final DriveAndAimAtTargetCommand innerCommand;

        AimAtTargetWithAllianceCommand(
                Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Translation2d blueAllianceTarget) {
            this.drive = drive;
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.blueAllianceTarget = blueAllianceTarget;
            this.innerCommand = new DriveAndAimAtTargetCommand(drive, xSupplier, ySupplier, () -> {
                // Mirror target position for red alliance
                if (DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red) {
                    // Mirror across field center (assuming field is 16.54m x 8.07m)
                    return new Translation2d(16.54 - blueAllianceTarget.getX(), blueAllianceTarget.getY());
                }
                return blueAllianceTarget;
            });

            addRequirements(drive);
        }

        @Override
        public void initialize() {
            innerCommand.initialize();
        }

        @Override
        public void execute() {
            innerCommand.execute();
        }

        @Override
        public void end(boolean interrupted) {
            innerCommand.end(interrupted);
        }

        @Override
        public boolean isFinished() {
            return innerCommand.isFinished();
        }
    }

    /**
     * Command that aims at the hub using vision AprilTag detection.
     *
     * <p>Uses the vision system to find the nearest visible hub AprilTag and aims at it.
     */
    private static class AimAtHubWithVisionCommand extends Command {
        private final Drive drive;
        private final Vision vision;
        private final DoubleSupplier xSupplier;
        private final DoubleSupplier ySupplier;
        private final Translation2d fallbackTarget;
        private final DriveAndAimAtTargetCommand innerCommand;

        AimAtHubWithVisionCommand(
                Drive drive,
                Vision vision,
                DoubleSupplier xSupplier,
                DoubleSupplier ySupplier,
                Translation2d fallbackTarget) {
            this.drive = drive;
            this.vision = vision;
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.fallbackTarget = fallbackTarget;
            this.innerCommand = new DriveAndAimAtTargetCommand(drive, xSupplier, ySupplier, () -> {
                // Check if red alliance
                boolean isRed = DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;

                // Try to get hub position from vision
                var hubPosition = vision.getVisibleHubPosition(isRed);

                if (hubPosition.isPresent()) {
                    return hubPosition.get();
                }

                // Fallback to fixed target if no hub visible
                if (isRed) {
                    return new Translation2d(16.54 - fallbackTarget.getX(), fallbackTarget.getY());
                }
                return fallbackTarget;
            });

            addRequirements(drive, vision);
        }

        @Override
        public void initialize() {
            innerCommand.initialize();
        }

        @Override
        public void execute() {
            innerCommand.execute();
        }

        @Override
        public void end(boolean interrupted) {
            innerCommand.end(interrupted);
        }

        @Override
        public boolean isFinished() {
            return innerCommand.isFinished();
        }
    }
}
