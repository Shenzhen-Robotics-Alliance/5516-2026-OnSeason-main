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

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "frontLeftCam";
    public static String camera1Name = "frontRightCam";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 = new Transform3d(
            Inches.of(27.5).div(2), // Forward 27.5/2 inch
            Inches.of(24.25).div(2), // Leftward 24.25/2 inch
            Inches.of(2).plus(Inches.of(4)), // Height 2 inch + Chassis Height 4 inch
            new Rotation3d(Degrees.zero(), Degrees.of(-24), Degrees.zero())); // Pitch upwards 24 degrees
    public static Transform3d robotToCamera1 = new Transform3d(
            Inches.of(27.5).div(2), // Forward 27.5/2 inch
            Inches.of(24.25).div(2).times(-1), // Rightwards 24.25/2 inch
            Inches.of(2).plus(Inches.of(4)), // Height 2 inch + Chassis Height 4 inch
            new Rotation3d(Degrees.zero(), Degrees.of(-24), Degrees.zero())); // Pitch upwards 24 degrees

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
