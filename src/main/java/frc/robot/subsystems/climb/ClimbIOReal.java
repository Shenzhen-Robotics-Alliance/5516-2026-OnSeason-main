package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.climb.ClimbConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public final class ClimbIOReal implements ClimbIO {
    private final TalonFX climbMotor;
    private final CANcoder absoluteEncoder;

    private final StatusSignal<Angle> absoluteEncoderPosition;
    private final StatusSignal<Current> climbCurrent;

    public ClimbIOReal() {
        // Initialize hardware
        this.climbMotor = new TalonFX(CLIMB_MOTOR_ID);
        this.absoluteEncoder = new CANcoder(CANCODER_ID);

        // Configure absolute encoder
        absoluteEncoder
                .getConfigurator()
                .apply(new MagnetSensorConfigs()
                        .withSensorDirection(
                                CANCODER_INVERTED
                                        ? SensorDirectionValue.Clockwise_Positive
                                        : SensorDirectionValue.CounterClockwise_Positive));

        // Configure motor
        climbMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        climbMotor
                .getConfigurator()
                .apply(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(CLIMB_CURRENT_LIMIT.in(Amps))
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(60));

        // Get status signals
        absoluteEncoderPosition = absoluteEncoder.getAbsolutePosition();
        climbCurrent = climbMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, absoluteEncoderPosition, climbCurrent);
        absoluteEncoder.optimizeBusUtilization();
    }

    private final VoltageOut voltageOut = new VoltageOut(0);

    @Override
    public void updateInputs(ClimbInputs inputs) {
        // Get absolute encoder reading
        StatusCode statusCode = BaseStatusSignal.refreshAll(absoluteEncoderPosition);
        inputs.absoluteEncoderConnected = absoluteEncoder.isConnected();
        inputs.climbAbsolutePosition = absoluteEncoderPosition.getValueAsDouble();

        // Get motor current
        statusCode = BaseStatusSignal.refreshAll(climbCurrent);
        inputs.hardwareConnected = statusCode.isOK();
        inputs.climbCurrentAmps = climbCurrent.getValueAsDouble();
    }

    @Override
    public void setMotorOutput(double volts) {
        climbMotor.setControl(voltageOut.withOutput(volts));
    }
}
