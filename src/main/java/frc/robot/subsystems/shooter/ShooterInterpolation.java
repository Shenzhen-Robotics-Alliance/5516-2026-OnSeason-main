package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.HubConstants;

public class ShooterInterpolation {
    // Use Hub center from HubConstants (consistent across the codebase)
    public static final Translation2d HUB_CENTER = HubConstants.BLUE_HUB_CENTER;

    // table for left and right shooter RPMs based on distance to hub
    private static final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

    static {
        shooterMap.put(1.5, 3000.0);
        shooterMap.put(2.5, 4500.0);
        shooterMap.put(3.5, 5500.0);
        shooterMap.put(4.5, 6300.0);
    }

    public record InterpolatedRPM(double shooterRPM, double distance) {}

    public static InterpolatedRPM calculate(Pose2d robotPose) {
        double distance = robotPose.getTranslation().getDistance(HUB_CENTER);
        return new InterpolatedRPM(shooterMap.get(distance), distance);
    }

    public static double getRpm(double distance) {
        return shooterMap.get(distance);
    }
}
