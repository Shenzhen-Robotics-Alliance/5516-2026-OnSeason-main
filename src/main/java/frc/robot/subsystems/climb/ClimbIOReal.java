package frc.robot.subsystems.climb;

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
    private final TalonFX supportMotor;
    private final CANcoder supportEncoder = new CANcoder(21);
    private final CANcoder climbEncoder = new CANcoder(20);
    private final StatusSignal<Angle> climbPosition;
    private final StatusSignal<Current> climbCurrent;
    private final StatusSignal<Angle> supportPosition;
    private final StatusSignal<Current> supportCurrent;

    public ClimbIOReal() {
        this.climbMotor = new TalonFX(13);
        this.supportMotor = new TalonFX(28);
        climbEncoder
                .getConfigurator()
                .apply(new MagnetSensorConfigs()
                        .withMagnetOffset(-0.1)
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
        supportEncoder
                .getConfigurator()
                .apply(new MagnetSensorConfigs()
                        .withMagnetOffset(-0.1)
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

        climbMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        supportMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        climbMotor
                .getConfigurator()
                .apply(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40)
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(60));
        supportMotor
                .getConfigurator()
                .apply(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40)
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(60));
        climbPosition = climbEncoder.getPosition();
        supportPosition = supportEncoder.getPosition();
        climbCurrent = climbMotor.getSupplyCurrent();
        supportCurrent = climbMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, climbPosition, climbCurrent, supportPosition, supportCurrent);
    }

    private final VoltageOut voltageOut = new VoltageOut(0);

    @Override
    public void updateInputs(ClimbInputs inputs) {
        StatusCode statusCode =
                BaseStatusSignal.refreshAll(climbPosition, climbCurrent, supportPosition, supportCurrent);
        inputs.hardwareConnected = statusCode.isOK();

        inputs.climbAbsolutePosition = climbPosition.getValueAsDouble();
        inputs.climbCurrentAmps = climbCurrent.getValueAsDouble();
        inputs.climbAbsolutePosition = climbPosition.getValueAsDouble();
        inputs.climbCurrentAmps = climbCurrent.getValueAsDouble();
    }

    @Override
    public void setMotorOutput(double volts) {
        climbMotor.setControl(voltageOut.withOutput(volts));
        supportMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setSupportMotorOutput(double volts) {
        supportMotor.setControl(voltageOut.withOutput(volts));
    }
}
