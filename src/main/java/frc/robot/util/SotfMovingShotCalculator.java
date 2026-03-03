package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Moving-shot calculator based on the FTC "Hood Angle and Velocity Calcs" document.
 * Units: meters, meters/sec, radians.
 */
public final class SotfMovingShotCalculator {
    public static final class Config {
        // Physics constants
        public double gravity = 9.80665; // m/s^2

        // Field geometry
        public double launchHeight = 0.0; // m
        public double goalHeight = 0.0; // m

        // Desired trajectory angle at the goal (rad). Negative is downward.
        public double entryAngleRad = Math.toRadians(-45.0);

        // Mechanical limits for launch angle (rad)
        public double minLaunchAngleRad = Math.toRadians(20.0);
        public double maxLaunchAngleRad = Math.toRadians(65.0);

        // Hood servo calibration: angle -> servo mapping
        public double hoodAngleMinRad = Math.toRadians(20.0); // a1
        public double hoodAngleMaxRad = Math.toRadians(65.0); // a2
        public double hoodServoMin = 0.0; // s1
        public double hoodServoMax = 1.0; // s2

        // Flywheel calibration: rpm = slope * launchSpeed + intercept
        public double flywheelSlope = 1.0;
        public double flywheelIntercept = 0.0;
        public double flywheelMinRpm = 0.0;
        public double flywheelMaxRpm = 6000.0;
    }

    public static final class Result {
        public final boolean valid;
        public final String error;

        public final double distanceM;
        public final double launchAngleRad;
        public final double launchSpeedMps;
        public final double compensatedAngleRad;
        public final double compensatedSpeedMps;
        public final double flightTimeSec;
        public final double radialVelMps;
        public final double tangentialVelMps;
        public final double vxCompMps;
        public final double vxNewMps;
        public final double headingOffsetRad;
        public final double hoodServoPos;
        public final double flywheelRpm;

        private Result(
                boolean valid,
                String error,
                double distanceM,
                double launchAngleRad,
                double launchSpeedMps,
                double compensatedAngleRad,
                double compensatedSpeedMps,
                double flightTimeSec,
                double radialVelMps,
                double tangentialVelMps,
                double vxCompMps,
                double vxNewMps,
                double headingOffsetRad,
                double hoodServoPos,
                double flywheelRpm) {
            this.valid = valid;
            this.error = error;
            this.distanceM = distanceM;
            this.launchAngleRad = launchAngleRad;
            this.launchSpeedMps = launchSpeedMps;
            this.compensatedAngleRad = compensatedAngleRad;
            this.compensatedSpeedMps = compensatedSpeedMps;
            this.flightTimeSec = flightTimeSec;
            this.radialVelMps = radialVelMps;
            this.tangentialVelMps = tangentialVelMps;
            this.vxCompMps = vxCompMps;
            this.vxNewMps = vxNewMps;
            this.headingOffsetRad = headingOffsetRad;
            this.hoodServoPos = hoodServoPos;
            this.flywheelRpm = flywheelRpm;
        }

        private static Result invalid(String error, double distanceM) {
            return new Result(
                    false,
                    error,
                    distanceM,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN);
        }
    }

    private SotfMovingShotCalculator() {}

    public static Result solve(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Translation2d targetPosition,
            Config cfg) {
        if (robotPose == null || targetPosition == null || cfg == null) {
            return Result.invalid("null input", Double.NaN);
        }

        double dx = targetPosition.getX() - robotPose.getX();
        double dy = targetPosition.getY() - robotPose.getY();
        double distance = Math.hypot(dx, dy);
        if (distance < 1e-6) {
            return Result.invalid("distance too small", distance);
        }

        double lineHeading = Math.atan2(dy, dx);

        Translation2d fieldVelocity = new Translation2d(
                        robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
                .rotateBy(robotPose.getRotation());
        double vMag = fieldVelocity.getNorm();
        double vHeading = Math.atan2(fieldVelocity.getY(), fieldVelocity.getX());
        double delta = vHeading - lineHeading;

        double vRadial = -Math.cos(delta) * vMag;
        double vTangential = Math.sin(delta) * vMag;

        double y = cfg.goalHeight - cfg.launchHeight;
        double tanAlpha = (2.0 * y / distance) - Math.tan(cfg.entryAngleRad);
        double alpha = Math.atan(tanAlpha);
        double cosAlpha = Math.cos(alpha);
        if (Math.abs(cosAlpha) < 1e-6) {
            return Result.invalid("cos(alpha) ~ 0", distance);
        }

        double denom = distance * tanAlpha - y;
        if (denom <= 1e-6) {
            return Result.invalid("invalid ballistic denom", distance);
        }

        double v0 = Math.sqrt(cfg.gravity * distance * distance / (2.0 * cosAlpha * cosAlpha * denom));
        if (!isFinite(v0)) {
            return Result.invalid("invalid v0", distance);
        }

        double t = distance / (v0 * cosAlpha);
        if (!isFinite(t) || t <= 1e-6) {
            return Result.invalid("invalid time", distance);
        }

        double vXComp = distance / t + vRadial;
        double vXNew = Math.hypot(vXComp, vTangential);
        double vY = v0 * Math.sin(alpha);

        double compensatedAngle = Math.atan2(vY, vXNew);
        double clampedAngle = MathUtil.clamp(
                compensatedAngle, cfg.minLaunchAngleRad, cfg.maxLaunchAngleRad);

        double xNew = vXNew * t;
        double tanClamped = Math.tan(clampedAngle);
        double denom2 = xNew * tanClamped - y;
        if (denom2 <= 1e-6) {
            return Result.invalid("invalid compensated denom", distance);
        }

        double v0New = Math.sqrt(
                cfg.gravity * xNew * xNew / (2.0 * Math.cos(clampedAngle) * Math.cos(clampedAngle) * denom2));
        if (!isFinite(v0New)) {
            return Result.invalid("invalid v0New", distance);
        }

        double headingOffset = Math.atan2(vTangential, vXComp);
        double hoodServo = mapAngleToServo(clampedAngle, cfg);
        double rpm = cfg.flywheelSlope * v0New + cfg.flywheelIntercept;
        rpm = MathUtil.clamp(rpm, cfg.flywheelMinRpm, cfg.flywheelMaxRpm);

        return new Result(
                true,
                "",
                distance,
                alpha,
                v0,
                clampedAngle,
                v0New,
                t,
                vRadial,
                vTangential,
                vXComp,
                vXNew,
                headingOffset,
                hoodServo,
                rpm);
    }

    private static double mapAngleToServo(double angleRad, Config cfg) {
        double a1 = cfg.hoodAngleMinRad;
        double a2 = cfg.hoodAngleMaxRad;
        double s1 = cfg.hoodServoMin;
        double s2 = cfg.hoodServoMax;

        if (Math.abs(a1 - a2) < 1e-6) {
            return MathUtil.clamp(s1, 0.0, 1.0);
        }

        double servo = ((s1 - s2) / (a1 - a2)) * (angleRad - a1) + s1;
        double minS = Math.min(s1, s2);
        double maxS = Math.max(s1, s2);
        return MathUtil.clamp(servo, minS, maxS);
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
