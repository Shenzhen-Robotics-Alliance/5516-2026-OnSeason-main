# Shooting on the Fly: 3 Implementation Plans

This document outlines three implementation paths for "shooting on the fly" (SOTF). All plans assume **no turret** and a **fast swerve drivetrain**, so the robot must actively control **heading** while moving. Each plan increases in complexity and performance.

---

## Plan A - Minimal Viable SOTF (Heading-Only + Empirical Shooter Map)

**Goal:** Get reliable on-the-fly shooting quickly using only heading control and a distance-based shooter map.

**Core idea:**
- Use odometry/vision target position to compute a desired **robot heading** to the goal.
- Use existing drive heading controllers to rotate the chassis while still allowing translation.
- Use a **distance-to-RPM/arm-angle map** (interpolated table) instead of physics.

**Implementation steps:**
1. **Target selection:** use fixed field target position (or vision hub position) to compute bearing.
2. **Heading control:** call `DriveCommands.joystickDriveAtAngle(...)` with the target bearing.
3. **Shooter map:** create a lookup table (distance -> shooter RPM and arm angle).
4. **Shot gating:** only allow feeding when:
   - heading error < threshold
   - shooter RPM within tolerance
   - arm angle within tolerance

**Why it fits the current repo:**
- Drive heading control already exists.
- Shooter subsystem already accepts RPM commands.
- Requires minimal new math.

**Pros:**
- Fast to build and tune.
- Low risk.
- Good enough for moderate speed shots.

**Cons:**
- No velocity compensation; accuracy degrades at higher speeds.
- Requires more strict speed limits while shooting.

**When to choose:**
- You need a fast, reliable baseline.
- You can slow down slightly during shots.

---

## Plan B - FTC-Style Moving Shot Solver (Velocity-Compensated Heading)

**Goal:** Port the FTC moving-shot model to FRC and compensate for robot velocity to improve accuracy while moving.

**Core idea:**
- Use robot pose + chassis velocity + target pose.
- Solve ballistics for launch speed and time-of-flight.
- Convert **turret lead** into a **robot heading offset** (since no turret).
- Use solver outputs to drive shooter RPM and arm angle.

**Implementation steps:**
1. **Port solver:** add a FRC version of the FTC `ShooterMovingShotCalculator`.
   - Convert to meters / meters-per-sec.
2. **Compute lead:** the solver's turret offset becomes a **heading offset** for the robot.
3. **Drive control:** feed desired heading into `joystickDriveAtAngle` or a new "SOTF drive" command.
4. **Shooter control:** set RPM and arm angle from solver outputs.
5. **Shot gating:** same as Plan A, but based on solver outputs.

**Pros:**
- True velocity compensation.
- Better accuracy at higher drive speeds.
- Uses physics instead of tuning-only tables.

**Cons:**
- Requires correct calibration of shooter model and arm kinematics.
- More tuning effort.

**When to choose:**
- You want reliable shots without slowing the robot.
- You can invest time in tuning and validation.

---

## Plan C - Predictive SOTF with Vision Fusion + Time-of-Flight

**Goal:** Best-possible accuracy while moving fast, using vision + odometry fusion and predictive timing.

**Core idea:**
- Fuse pose from odometry + AprilTag vision for best target alignment.
- Predict robot pose at **shot exit time** (includes shooter spin-up + ball flight time).
- Use solver to aim at **predicted pose**, not current pose.
- Optionally gate shots when the robot is within a velocity/accel envelope.

**Implementation steps:**
1. **Pose fusion:** use Vision's target position if visible, else fall back to odometry.
2. **Time-of-flight prediction:** compute ball flight time + feeder/exit delay.
3. **Predictive heading:** aim at target based on predicted robot pose.
4. **Dynamic shot gating:** allow shot only when heading error + predicted error are within limits.
5. **Auto tuning hooks:** log all predicted vs. actual error for offline tuning.

**Pros:**
- Highest accuracy at speed.
- Strongest resilience to vision dropout.

**Cons:**
- Most complex to implement and tune.
- More dependence on accurate timing and model parameters.

**When to choose:**
- You need max performance late season.
- You already have stable vision + odometry.

---

## Suggested Decision Path

1. Start with **Plan A** to validate shooter map, gating logic, and driver workflow.
2. Upgrade to **Plan B** once basic SOTF is stable.
3. If required by game demands, move to **Plan C** for max performance.

---

## Tuning Checklist

**Geometry and units**
- Confirm all solver inputs are in meters, meters per second, and radians.
- Verify field target location is correct for both alliances.
- Measure launch height and goal height relative to the same origin.

**Shooter model**
- Build an RPM-to-muzzle-speed calibration (or fit a linear model).
- Validate hood/arm angle mapping vs. measured launch angle.
- Clamp and test min/max RPM and arm angle ranges.

**Drive and pose**
- Validate gyro direction and field-relative transforms.
- Confirm odometry drift stays within your shot tolerance.
- Log chassis speed vs. commanded speed at multiple drive speeds.

**Velocity compensation (Plan B/C)**
- Test straight-line shots at multiple speeds (forward/backward).
- Test strafing shots (left/right) for tangential lead correctness.
- Tune heading controller gains for fast but stable alignment.

**Gating thresholds**
- Set heading error tolerance (deg) for shot release.
- Set RPM tolerance and arm-angle tolerance for shot release.
- Add a velocity or acceleration limit if needed for accuracy.

**Vision and fallback**
- Validate vision target selection and alliance mirroring.
- Log timestamps and latency if using vision pose updates.
- Ensure safe fallback to odometry when vision drops.

---

## Notes for This Repo

- **Heading control:** `DriveCommands.joystickDriveAtAngle(...)` is the easiest integration point.
- **Velocity source:** `Drive.getChassisSpeeds()` is already available.
- **Pose source:** `Drive.getPose()` and vision-based target selection exist.
- **Shooter control:** `Shooter.runShooterVelocity(...)` is ready for dynamic RPM.

---

## Ballistics Equations (Obsidian LaTeX)

**A) Required entry angle at the goal**

Given target distance \(x\), height \(y\), desired entry angle \(\theta\), and gravity \(g\):

$$
v_x = v_0 \cos \alpha,\quad v_y = v_0 \sin \alpha - g t
$$

$$
x = v_0 \cos \alpha \, t,\quad y = v_0 \sin \alpha \, t - \frac{1}{2} g t^2
$$

$$
\tan \theta = \frac{v_y}{v_x}
$$

Eliminate time using \(t = x / (v_0 \cos \alpha)\):

$$
\tan \theta = \tan \alpha - \frac{g x}{v_0^2 \cos^2 \alpha}
$$

Solve for launch angle:

$$
\tan \alpha = \frac{2y}{x} - \tan \theta
$$

$$
\alpha = \arctan\left(\frac{2y}{x} - \tan \theta\right)
$$

Solve for launch speed:

$$
v_0 = \sqrt{\frac{g x^2}{2 \cos^2 \alpha (x \tan \alpha - y)}}
$$

**B) Velocity compensation (robot moving)**

Let the robot's field velocity magnitude be \(V_{mag}\), and let \(\\Delta = \\theta_{vel} - \\theta_{line}\).

$$
V_{radial} = -\cos(\Delta) V_{mag}
$$

$$
V_{tangential} = \sin(\Delta) V_{mag}
$$

Flight time:

$$
t = \frac{x}{v_0 \cos \alpha}
$$

Compensated x velocity and new x magnitude:

$$
V_{x,comp} = \frac{x}{t} + V_{radial}
$$

$$
V_{x,new} = \sqrt{V_{x,comp}^2 + V_{tangential}^2}
$$

New launch angle:

$$
V_y = v_0 \sin \alpha
$$

$$
\alpha_{new} = \arctan\left(\frac{V_y}{V_{x,new}}\right)
$$

Recompute launch speed using the same equation as above with:

$$
x_{new} = V_{x,new} \cdot t
$$

Heading lead (turret offset in FTC, chassis heading offset in FRC):

$$
\text{headingOffset} = \arctan\left(\frac{V_{tangential}}{V_{x,comp}}\right)
$$

**C) Mechanism mapping**

Hood servo mapping between two calibration points \((a_1, s_1)\) and \((a_2, s_2)\):

$$
\text{servo} = \left(\frac{s_1 - s_2}{a_1 - a_2}\right) (\alpha - a_1) + s_1
$$

Flywheel launch speed from range test:

$$
R = \frac{v_0^2 \sin(2\theta)}{g}
$$

$$
v_0 = \sqrt{\frac{R g}{\sin(2\theta)}}
$$

Then fit:

$$
\text{RPM} = m v_0 + b
$$

---

## Sample Code (Plan B - FTC-Style Moving Shot Solver)

These snippets are based on the FTC moving-shot logic, adapted to WPILib types and no turret. This is intentionally minimal and should be integrated with your existing Drive and Shooter patterns.

### 1) Moving-shot solver (meters, meters per second, radians)

```java
// Use SotfMovingShotCalculator in src/main/java/frc/robot/util
```

### 2) Command skeleton (drive + aim + shooter control)

```java
package frc.robot.commands.sotf;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.SotfMovingShotCalculator;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShootOnTheFlyCommand extends Command {
    private final Drive drive;
    private final Shooter shooter;
    private final Arm arm;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final Supplier<Translation2d> targetSupplier;
    private final SotfMovingShotCalculator.Config cfg;
    private final ProfiledPIDController headingController =
            new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(12.0, 20.0));

    public ShootOnTheFlyCommand(
            Drive drive,
            Shooter shooter,
            Arm arm,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Translation2d> targetSupplier,
            SotfMovingShotCalculator.Config cfg) {
        this.drive = drive;
        this.shooter = shooter;
        this.arm = arm;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.targetSupplier = targetSupplier;
        this.cfg = cfg;
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive, shooter, arm);
    }

    @Override
    public void initialize() {
        headingController.reset(drive.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Translation2d target = targetSupplier.get();
        var result = SotfMovingShotCalculator.solve(drive.getPose(), drive.getChassisSpeeds(), target, cfg);
        if (!result.valid) {
            drive.stop();
            shooter.runShooterVelocity(0.0);
            return;
        }

        // Desired heading is target bearing + lead (no turret)
        Translation2d delta = target.minus(drive.getPose().getTranslation());
        double targetHeading = Math.atan2(delta.getY(), delta.getX()) + result.leadAngleRad;

        // Linear drive from joysticks (same style as DriveCommands)
        double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
        double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);
        Translation2d linearVelocity = new Translation2d(x, y).times(drive.getMaxLinearSpeedMetersPerSec());

        double omega = headingController.calculate(drive.getRotation().getRadians(), targetHeading);
        ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

        boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));

        // Shooter and arm commands
        // TODO: add a public setter in Shooter (ex: requestRpm) or wrap in a command factory
        shooter.requestRpm(result.flywheelRpm);
        arm.requestPosition(ArmConstants.launchAngleToSetpoint(result.launchAngleRad));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        shooter.requestRpm(0.0);
    }
}
```

Notes:
- Replace `ArmConstants.launchAngleToSetpoint(...)` with your actual arm/hood mapping.
- Add `Shooter.requestRpm(...)` (or similar) if you prefer direct setters in a custom command.
- Add gating logic (heading error, RPM error, arm error) before feeding.
- Use Vision target if available; fall back to a fixed field position otherwise.
