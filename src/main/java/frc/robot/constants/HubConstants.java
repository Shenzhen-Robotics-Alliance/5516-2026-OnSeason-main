// Copyright 2026 FRC 5516
// Shenzhen Robotics Alliance
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

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Constants for the Hub (2026 Reefscape).
 *
 * <p>The Hub is the central scoring target. The AprilTags are placed around the Hub for vision localization, but the
 * actual aiming target should be the Hub center.
 */
public final class HubConstants {
    private HubConstants() {}

    /** Hub center position for Blue Alliance (in meters). */
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6, 4.01);

    /** Hub center position for Red Alliance (mirrored from Blue, in meters). */
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.94, 4.01);

    /** Field length in meters (2026 Reefscape). */
    public static final double FIELD_LENGTH_METERS = 16.54;

    /** Field width in meters (2026 Reefscape). */
    public static final double FIELD_WIDTH_METERS = 8.02;

    /**
     * Gets the Hub center position based on the current alliance.
     *
     * @return Hub center position for the current alliance
     */
    public static Translation2d getHubCenter() {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red) {
            // Mirror across field center for Red alliance
            return new Translation2d(FIELD_LENGTH_METERS - BLUE_HUB_CENTER.getX(), BLUE_HUB_CENTER.getY());
        }
        return BLUE_HUB_CENTER;
    }

    /**
     * Gets the Hub center position for a specific alliance.
     *
     * @param isRedAlliance True if Red alliance
     * @return Hub center position for the specified alliance
     */
    public static Translation2d getHubCenter(boolean isRedAlliance) {
        if (isRedAlliance) {
            return new Translation2d(FIELD_LENGTH_METERS - BLUE_HUB_CENTER.getX(), BLUE_HUB_CENTER.getY());
        }
        return BLUE_HUB_CENTER;
    }
}
