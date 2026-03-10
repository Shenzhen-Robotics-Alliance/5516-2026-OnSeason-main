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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HubConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Commands for aligning the robot to the hub with velocity prediction.
 *
 * <p>This provides feedforward + feedback control for more accurate aiming when the robot is moving.
 *
 * <p>Note: The Hub center position is fixed and does not depend on AprilTag detection. AprilTags are used for robot
 * pose estimation (odometry), but aiming is done using the fixed Hub center coordinates for stability and reliability.
 */
public class HubAlignmentCommands {
    // PID constants for angle control
    private static final double ANGLE_KP = 8.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 20.0;
    private static final double ANGLE_MAX_ACCELERATION = 30.0;

    // Feedforward gain for velocity prediction (0.7 = 70% feedforward)
    public static final double FEED_FORWARD_RATE = 0.7;

    // Robot control period (20ms)
    private static final double ROBOT_PERIOD_SECS = 0.02;

    // Deadband for joystick (same as DriveCommands)
    private static final double DEADBAND = 0.1;

    private HubAlignmentCommands() {}

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
     * Creates a command that aims at the hub center using fixed coordinates.
     *
     * <p>Uses the fixed Hub center position from HubConstants, automatically handling alliance color (blue/red) by
     * mirroring coordinates. This does NOT use AprilTag positions for aiming - AprilTags are only used for robot pose
     * estimation.
     *
     * @param drive The drive subsystem
     * @param xSupplier Joystick X input
     * @param ySupplier Joystick Y input
     * @return The command that drives while aiming at the Hub center
     */
    public static Command aimAtHub(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return new AimAtHubCommand(drive, xSupplier, ySupplier);
    }

    /**
     * Creates a command that aims at the hub with velocity prediction.
     *
     * <p>This is the primary command for auto-aligning to the Hub during teleop. Uses fixed Hub center coordinates and
     * does not rely on vision AprilTag detection.
     *
     * @param drive The drive subsystem
     * @param xSupplier Joystick X input
     * @param ySupplier Joystick Y input
     * @return The command that drives while aiming at the Hub center
     */
    public static Command aimAtHubWithVelocityPrediction(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return new AimAtHubCommand(drive, xSupplier, ySupplier);
    }

    /** Get linear velocity from joysticks - same logic as DriveCommands for consistency. */
    private static Translation2d getLinearVelocityFromJoysticks(double x, double y, double maxSpeed) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude * maxSpeed, 0.0, new Rotation2d()))
                .getTranslation();
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
            // Get linear velocity from joysticks - negate x and y to match manual drive direction
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
     * Command that aims at the Hub center using fixed coordinates.
     *
     * <p>This is the optimized version that uses HubConstants for the target position, not AprilTag detection. The
     * AprilTags are still used by the Vision subsystem for robot pose estimation, but they do not affect aiming
     * direction.
     */
    private static class AimAtHubCommand extends Command {
        private final Drive drive;
        private final DoubleSupplier xSupplier;
        private final DoubleSupplier ySupplier;
        private final DriveAndAimAtTargetCommand innerCommand;

        AimAtHubCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
            this.drive = drive;
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.innerCommand = new DriveAndAimAtTargetCommand(drive, xSupplier, ySupplier, () -> {
                // Use fixed Hub center coordinates from HubConstants
                // Automatically handles alliance color (blue/red) by mirroring
                return HubConstants.getHubCenter();
            });

            addRequirements(drive);
        }

        @Override
        public void initialize() {
            Logger.recordOutput("HubAlignment/Mode", "FixedHubCenter");
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
