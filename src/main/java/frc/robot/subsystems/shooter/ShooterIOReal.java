package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterContants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {
    // Arrays for shooter motors
    private final TalonFX[] shooterMotors;
    private final StatusSignal<Current>[] shooterMotorCurrents;
    private final StatusSignal<Voltage>[] shooterMotorOutputVoltages;
    private final StatusSignal<AngularVelocity>[] shooterMotorVelocities;

    // Arrays for feeder motors
    private final TalonFX[] feederMotors;
    private final StatusSignal<Current>[] feederMotorCurrents;
    private final StatusSignal<Voltage>[] feederMotorOutputVoltages;
    private final StatusSignal<AngularVelocity>[] feederMotorVelocities;

    // Control requests
    private final VoltageOut voltageOut = new VoltageOut(Volts.zero());
    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);
    private final VelocityVoltage feederVelocityRequest = new VelocityVoltage(0);

    // Track current control mode
    private boolean shooterVelocityControl = false;
    private boolean feederVelocityControl = false;
    private double shooterTargetRPM = 0.0;
    private double feederTargetRPM = 0.0;

    public ShooterIOReal() {
        double freq = 100;

        // Initialize shooter motors
        int shooterCount = SHOOTERHARDWARE_CONSTANTS.shooterMotorIDs().length;
        shooterMotors = new TalonFX[shooterCount];
        shooterMotorCurrents = new StatusSignal[shooterCount];
        shooterMotorOutputVoltages = new StatusSignal[shooterCount];
        shooterMotorVelocities = new StatusSignal[shooterCount];

        // Initialize feeder motors
        int feederCount = SHOOTERHARDWARE_CONSTANTS.feederMotorIDs().length;
        feederMotors = new TalonFX[feederCount];
        feederMotorCurrents = new StatusSignal[feederCount];
        feederMotorOutputVoltages = new StatusSignal[feederCount];
        feederMotorVelocities = new StatusSignal[feederCount];

        // Configure shooter motors
        for (int i = 0; i < shooterCount; i++) {
            int motorID = SHOOTERHARDWARE_CONSTANTS.shooterMotorIDs()[i];
            boolean inverted = SHOOTERHARDWARE_CONSTANTS.shooterMotorInverted()[i];

            shooterMotors[i] = new TalonFX(motorID);

            // Apply current limit
            CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(SHOOTER_MOTORS_CURRENT_LIMIT);
            shooterMotors[i].getConfigurator().apply(currentLimit);

            // Apply motor output config
            MotorOutputConfigs outputConfig = new MotorOutputConfigs()
                    .withInverted(inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    // .withNeutralMode(NeutralModeValue.Brake);
                    .withNeutralMode(NeutralModeValue.Coast);
            shooterMotors[i].getConfigurator().apply(outputConfig);

            // Configure PID for velocity control on first motor (leader)
            if (i == 0) {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.Slot0 = SHOOTER_VELOCITY_GAINS;
                shooterMotors[i].getConfigurator().apply(config);
            }

            // Create status signals
            shooterMotorCurrents[i] = shooterMotors[i].getSupplyCurrent();
            shooterMotorOutputVoltages[i] = shooterMotors[i].getMotorVoltage();
            shooterMotorVelocities[i] = shooterMotors[i].getVelocity();

            // Set update frequencies
            BaseStatusSignal.setUpdateFrequencyForAll(
                    freq, shooterMotorCurrents[i], shooterMotorOutputVoltages[i], shooterMotorVelocities[i]);

            shooterMotors[i].optimizeBusUtilization();
        }

        // Configure follower motors for shooter (motors 1..n follow motor 0)
        for (int i = 1; i < shooterCount; i++) {
            shooterMotors[i].setControl(new Follower(shooterMotors[0].getDeviceID(), MotorAlignmentValue.Opposed));
        }

        // Configure feeder motors
        for (int i = 0; i < feederCount; i++) {
            int motorID = SHOOTERHARDWARE_CONSTANTS.feederMotorIDs()[i];
            boolean inverted = SHOOTERHARDWARE_CONSTANTS.feederMotorInverted()[i];

            feederMotors[i] = new TalonFX(motorID);

            // Apply current limit
            CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(FEEDER_MOTORS_CURRENT_LIMIT);
            feederMotors[i].getConfigurator().apply(currentLimit);

            // Apply motor output config
            MotorOutputConfigs outputConfig = new MotorOutputConfigs()
                    .withInverted(inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    // .withNeutralMode(NeutralModeValue.Brake);
                    .withNeutralMode(NeutralModeValue.Coast);
            feederMotors[i].getConfigurator().apply(outputConfig);

            // Configure PID for velocity control on first motor (leader)
            if (i == 0) {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.Slot0 = FEEDER_VELOCITY_GAINS;
                feederMotors[i].getConfigurator().apply(config);
            }

            // Create status signals
            feederMotorCurrents[i] = feederMotors[i].getSupplyCurrent();
            feederMotorOutputVoltages[i] = feederMotors[i].getMotorVoltage();
            feederMotorVelocities[i] = feederMotors[i].getVelocity();

            // Set update frequencies
            BaseStatusSignal.setUpdateFrequencyForAll(
                    freq, feederMotorCurrents[i], feederMotorOutputVoltages[i], feederMotorVelocities[i]);

            feederMotors[i].optimizeBusUtilization();
        }

        // Configure follower motors for feeder (motors 1..n follow motor 0)
        for (int i = 1; i < feederCount; i++) {
            feederMotors[i].setControl(new Follower(feederMotors[0].getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update shooter inputs
        int shooterCount = shooterMotors.length;
        inputs.shootersConnected = new boolean[shooterCount];
        inputs.shooterMotorsVelocityRPM = new double[shooterCount];

        double shooterTotalVolts = 0.0;
        double shooterTotalCurrent = 0.0;

        for (int i = 0; i < shooterCount; i++) {
            // Refresh signals
            boolean connected = BaseStatusSignal.refreshAll(
                            shooterMotorCurrents[i], shooterMotorOutputVoltages[i], shooterMotorVelocities[i])
                    .isOK();

            inputs.shootersConnected[i] = connected;

            if (connected) {
                shooterTotalVolts += shooterMotorOutputVoltages[i].getValueAsDouble();
                shooterTotalCurrent += shooterMotorCurrents[i].getValueAsDouble();
                // Convert from rotations per second to RPM
                inputs.shooterMotorsVelocityRPM[i] = shooterMotorVelocities[i].getValueAsDouble() * 60.0;
            }
        }

        inputs.shooterMotorsAverageVolts = shooterCount > 0 ? shooterTotalVolts / shooterCount : 0.0;
        inputs.shooterMotorsTotalCurrentAmps = shooterTotalCurrent;

        // Update feeder inputs
        int feederCount = feederMotors.length;
        inputs.feedersConnected = new boolean[feederCount];
        inputs.feederMotorsVelocityRPM = new double[feederCount];

        double feederTotalVolts = 0.0;
        double feederTotalCurrent = 0.0;

        for (int i = 0; i < feederCount; i++) {
            // Refresh signals
            boolean connected = BaseStatusSignal.refreshAll(
                            feederMotorCurrents[i], feederMotorOutputVoltages[i], feederMotorVelocities[i])
                    .isOK();

            inputs.feedersConnected[i] = connected;

            if (connected) {
                feederTotalVolts += feederMotorOutputVoltages[i].getValueAsDouble();
                feederTotalCurrent += feederMotorCurrents[i].getValueAsDouble();
                // Convert from rotations per second to RPM
                inputs.feederMotorsVelocityRPM[i] = feederMotorVelocities[i].getValueAsDouble() * 60.0;
            }
        }

        inputs.feederMotorsAverageVolts = feederCount > 0 ? feederTotalVolts / feederCount : 0.0;
        inputs.feederMotorsTotalCurrentAmps = feederTotalCurrent;
    }

    @Override
    public void setShooterMotorsVoltage(double volts) {
        shooterVelocityControl = false;

        // Apply voltage to leader motor only (followers will follow)
        if (shooterMotors.length > 0) {
            voltageOut.withOutput(volts);
            shooterMotors[0].setControl(voltageOut);
        }
    }

    @Override
    public void setFeederMotorsVoltage(double volts) {
        feederVelocityControl = false;

        // Apply voltage to leader motor only (followers will follow)
        if (feederMotors.length > 0) {
            voltageOut.withOutput(volts);
            feederMotors[0].setControl(voltageOut);
        }
    }

    @Override
    public void setShooterVelocity(double rpm) {
        shooterVelocityControl = true;
        shooterTargetRPM = rpm;

        // Apply velocity control to leader motor only (followers will follow)
        if (shooterMotors.length > 0) {
            // Convert RPM to rotations per second for Phoenix6
            double rps = rpm / 60.0;
            shooterVelocityRequest.withVelocity(rps);
            shooterMotors[0].setControl(shooterVelocityRequest);
        }
    }

    @Override
    public void setFeederVelocity(double rpm) {
        feederVelocityControl = true;
        feederTargetRPM = rpm;

        // Apply velocity control to leader motor only (followers will follow)
        if (feederMotors.length > 0) {
            // Convert RPM to rotations per second for Phoenix6
            double rps = rpm / 60.0;
            feederVelocityRequest.withVelocity(rps);
            feederMotors[0].setControl(feederVelocityRequest);
        }
    }
}
