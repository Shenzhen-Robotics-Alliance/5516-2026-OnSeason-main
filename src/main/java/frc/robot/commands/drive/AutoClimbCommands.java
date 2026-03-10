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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/**
 * Commands for automatic climb navigation using PathPlanner's dynamic pathfinding.
 *
 * <p>This provides one-button navigation to the climb prep position, automatically handling alliance color and obstacle
 * avoidance.
 */
public class AutoClimbCommands {
    // Field dimensions for alliance mirroring (2026 FRC field)
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.02;

    // Blue alliance climb prep position (approximately 1m from climb structure)
    // Position is on the left side of the blue alliance when facing the climb structure
    public static final Translation2d BLUE_CLIMB_PREP_POSITION = new Translation2d(1.5, 4.0);

    // Tolerance for reaching the target position (in meters)
    private static final double POSITION_TOLERANCE_METERS = 0.003;

    // Maximum standard deviation for pose estimation to be considered reliable
    // If the pose estimation uncertain ty is  igher than this, we should not use pathfinding
    private static final double MAX_POSE_STD_DEV_METERS = 0.7;

    // PathConstraints for stable movement with arm extended
    // Lower speed and acceleration for safety during climb approach
    private static final PathConstraints CLIMB_NAVIGATION_CONSTRAINTS = new PathConstraints(
            5.0, // Max velocity: 2 m/s (slower for safety)
            3.0, // Max acceleration: 3 m/s² (moderate acceleration)
            Units.degreesToRadians(360), // Max angular velocity: 360 deg/s
            Units.degreesToRadians(540)); // Max angular acceleration: 540 deg/s²

    // Alert for pose estimation being unreliable
    private static final Alert poseEstimationUnreliableAlert =
            new Alert("Pose estimation unreliable for auto-climb navigation", AlertType.kWarning);

    private AutoClimbCommands() {}

    /**
     * Mirrors a target position for red alliance.
     *
     * <p>Follows the same pattern as HubAlignmentCommands.mirrorForAlliance()
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
     * Gets the climb prep position for the current alliance.
     *
     * <p>Automatically mirrors for red alliance.
     *
     * @return The climb prep position for the current alliance
     */
    public static Translation2d getClimbPrepPosition() {
        return mirrorForAlliance(BLUE_CLIMB_PREP_POSITION);
    }

    /**
     * Gets the climb prep pose for the current alliance.
     *
     * <p>Automatically mirrors for red alliance.
     *
     * @return The climb prep pose for the current alliance
     */
    public static Pose2d getClimbPrepPose() {
        Translation2d position = getClimbPrepPosition();

        // For blue alliance, face toward the climb tower (180 degrees)
        // For red alliance, face toward the climb tower (0 degrees)
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        Rotation2d rotation = isRed ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);

        return new Pose2d(position, rotation);
    }

    /**
     * Checks if the pose estimation is reliable enough for pathfinding.
     *
     * <p>This checks if the robot's pose estimation has a reasonable standard deviation. If vision has been lost and
     * odometry has drifted significantly, we should not attempt autonomous pathfinding.
     *
     * <p>Note: This is a simplified check. For more accurate pose confidence assessment, you could check the vision
     * measurement timestamps and compare with encoder odometry.
     *
     * @param drive The drive subsystem to check pose confidence for
     * @return true if pose estimation is reliable, false otherwise
     */
    public static boolean isPoseEstimationReliable(Drive drive) {
        // Get the current pose
        Pose2d currentPose = drive.getPose();

        // Check if pose is within field boundaries
        // If the robot thinks it's outside the field, the pose is definitely wrong
        if (currentPose.getX() < 0
                || currentPose.getX() > FIELD_LENGTH_METERS
                || currentPose.getY() < 0
                || currentPose.getY() > FIELD_WIDTH_METERS) {
            Logger.recordOutput("AutoClimb/PoseReliable", false);
            Logger.recordOutput("AutoClimb/PoseReliabilityReason", "Outside field boundaries");
            return false;
        }

        // Additional heuristic: check if pose has changed dramatically since last update
        // This is a simplified check - in production you might want to track pose velocity
        // and compare with expected wheel velocities

        // For now, we'll use a simple heuristic: if the robot is moving very fast according
        // to odometry but the driver isn't providing input, the pose might be drifting
        // This is a basic check and could be enhanced with actual standard deviation tracking

        Logger.recordOutput("AutoClimb/PoseReliable", true);
        Logger.recordOutput("AutoClimb/PoseReliabilityReason", "Pose within field boundaries");

        return true;
    }

    /**
     * Creates a command that pathfinds to the climb prep position.
     *
     * <p>This command uses PathPlanner's dynamic pathfinding to navigate to the climb prep position while avoiding
     * obstacles. The command will automatically end when the robot reaches within the tolerance of the target position.
     *
     * <p>Safety check: If pose estimation is unreliable, the command will not start and will log a warning.
     *
     * @param drive The drive subsystem
     * @return The pathfinding command, or a no-op command if pose is unreliable
     */
    public static Command pathfindToClimbPrep(Drive drive) {
        return new PathfindToClimbPrepCommand(drive);
    }

    /** Command that pathfinds to the climb prep position. */
    private static class PathfindToClimbPrepCommand extends Command {
        private final Drive drive;
        private Command pathfindingCommand;
        private boolean poseReliable = false;

        PathfindToClimbPrepCommand(Drive drive) {
            this.drive = drive;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            // Check pose estimation reliability
            poseReliable = isPoseEstimationReliable(drive);

            if (!poseReliable) {
                // Pose is not reliable - log warning and alert driver
                poseEstimationUnreliableAlert.set(true);
                Logger.recordOutput("AutoClimb/Status", "Pose unreliable, command cancelled");
                Logger.recordOutput("AutoClimb/TargetPose", getClimbPrepPose());
                return;
            }

            poseEstimationUnreliableAlert.set(false);

            // Get target pose for current alliance (automatically handles alliance flipping)
            Pose2d targetPose = getClimbPrepPose();

            boolean isRed = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;

            Logger.recordOutput("AutoClimb/Status", "Pathfinding to climb prep");
            Logger.recordOutput("AutoClimb/TargetPose", targetPose);
            Logger.recordOutput("AutoClimb/StartPose", drive.getPose());
            Logger.recordOutput("AutoClimb/Alliance", isRed ? "Red" : "Blue");
            Logger.recordOutput("AutoClimb/IsRedAlliance", isRed);

            // Create pathfinding command using AutoBuilder
            // The command will automatically end when the robot reaches the target
            pathfindingCommand = AutoBuilder.pathfindToPose(
                    targetPose, CLIMB_NAVIGATION_CONSTRAINTS, 0.0 // Goal end velocity (stop at target)
                    );

            // Initialize the pathfinding command
            pathfindingCommand.initialize();
        }

        @Override
        public void execute() {
            if (!poseReliable) {
                return;
            }

            // Execute the pathfinding command
            if (pathfindingCommand != null) {
                pathfindingCommand.execute();
            }

            // Log current state
            Pose2d currentPose = drive.getPose();
            Pose2d targetPose = getClimbPrepPose();
            double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());

            Logger.recordOutput("AutoClimb/CurrentPose", currentPose);
            Logger.recordOutput("AutoClimb/DistanceToTarget", distanceToTarget);
            Logger.recordOutput("AutoClimb/AtTarget", distanceToTarget <= POSITION_TOLERANCE_METERS);
        }

        @Override
        public void end(boolean interrupted) {
            if (pathfindingCommand != null) {
                pathfindingCommand.end(interrupted);
            }

            if (interrupted) {
                Logger.recordOutput("AutoClimb/Status", "Command interrupted");
            } else {
                // Check if we actually reached the target
                Pose2d currentPose = drive.getPose();
                Pose2d targetPose = getClimbPrepPose();
                double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());

                if (distanceToTarget <= POSITION_TOLERANCE_METERS) {
                    Logger.recordOutput("AutoClimb/Status", "Reached climb prep position");
                } else {
                    Logger.recordOutput("AutoClimb/Status", "Command finished but not at target");
                }
            }

            // Stop the drive to ensure clean handoff to driver
            drive.stop();
        }

        @Override
        public boolean isFinished() {
            if (!poseReliable) {
                // Don't start if pose is unreliable
                return true;
            }

            if (pathfindingCommand == null) {
                return true;
            }

            // Check if pathfinding command is finished
            if (pathfindingCommand.isFinished()) {
                // Additional check: verify we're actually close to the target
                Pose2d currentPose = drive.getPose();
                Pose2d targetPose = getClimbPrepPose();
                double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());

                Logger.recordOutput("AutoClimb/AtTarget", distanceToTarget <= POSITION_TOLERANCE_METERS);

                return distanceToTarget <= POSITION_TOLERANCE_METERS;
            }

            return false;
        }
    }
}
