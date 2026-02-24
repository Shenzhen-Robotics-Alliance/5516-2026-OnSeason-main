package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ArmIO {

    final class ArmInputs implements LoggableInputs {
        /**
         * The (optional) arm absolute encoder angle which is already calibrated. Empty if the absolute Enconder is
         * disconnected.
         */
        public Optional<Rotation2d> absoluteEncoderAngle;
        /** Whether the CAN communications between the rio and the motor are good. */
        public boolean armMotorConnected;
        /**
         * The relative encoder angle, measured by the relative encoder (inside the motor). Gearing is NOT considered.
         */
        public double relativeEncoderAngledRad;
        /** The relative encoder velocity, measured by the relative encoder. Gearing is NOT considered. */
        public double relativeEncoderVelocityRadPerSec;
        /** The supply current of the motor. */
        public double armMotorSupplyCurrentAmps;
        /** The actual output voltage of the motor. */
        public double armMotorOutputVolts;

        public boolean intakeMotorConnected;
        public double intakeMotorSupplyCurrentAmps;
        public double intakeMotorOutputVolts;

        public ArmInputs() {
            this.absoluteEncoderAngle = Optional.empty();
            this.armMotorConnected = false;
            this.relativeEncoderAngledRad = 0.0;
            this.relativeEncoderVelocityRadPerSec = 0.0;
            this.armMotorSupplyCurrentAmps = 0.0;
            this.armMotorOutputVolts = 0.0;

            this.intakeMotorConnected = false;
            this.intakeMotorSupplyCurrentAmps = 0.0;
            this.intakeMotorOutputVolts = 0.0;
        }

        @Override
        public void toLog(LogTable table) {
            // Arm Motor info.
            table.put("absoluteEncoderAnglePresent", absoluteEncoderAngle.isPresent());
            table.put("absoluteEncoderAngle", absoluteEncoderAngle.orElse(Rotation2d.kZero));
            table.put("armMotorConnected", armMotorConnected);
            table.put("relativeEncoerAngledRad", relativeEncoderAngledRad);
            table.put("encoderVelocityRadPerSec", relativeEncoderVelocityRadPerSec);
            table.put("armMotorSupplyCurrentAmps", armMotorSupplyCurrentAmps);
            table.put("armMotorOutputVolts", armMotorOutputVolts);
            // Intake Motor info.
            table.put("intakeMotorConnected", intakeMotorConnected);
            table.put("intakeMotorSupplyCurrentAmps", intakeMotorSupplyCurrentAmps);
            table.put("intakeMotorOutputVolts", intakeMotorOutputVolts);
        }

        @Override
        public void fromLog(LogTable table) {
            boolean absoluteEnconderAnglePresent = table.get("absoluteEncoderAnglePresent", false);
            absoluteEncoderAngle = absoluteEnconderAnglePresent
                    ? Optional.of(table.get("absoluteEncoderAngle", Rotation2d.kZero))
                    : Optional.empty();
            armMotorConnected = table.get("armMotorConnected", armMotorConnected);
            relativeEncoderAngledRad = table.get("relativeEncoerAngledRad", 0.0);
            relativeEncoderVelocityRadPerSec = table.get("encoderVelocityRadPerSec", 0.0);
            armMotorSupplyCurrentAmps = table.get("armMotorSupplyCurrentAmps", 0.0);
            armMotorOutputVolts = table.get("armMotorOutputVolts", 0.0);

            intakeMotorConnected = table.get("intakeMotorConnected", intakeMotorConnected);
            intakeMotorSupplyCurrentAmps = table.get("intakeMotorSupplyCurrentAmps", 0.0);
            intakeMotorOutputVolts = table.get("intakeMotorOutputVolts", 0.0);
        }
    }

    void updateInputs(ArmInputs armInputs);

    default void setArmMotorOutput(Voltage voltage) {}

    default void setArmMotorBrake(boolean brakeModeEnable) {}

    default void setIntakeMotorOutput(Voltage voltage) {}

    default void setIntakeMotorBrake(boolean brakeModeEnable) {}
}
