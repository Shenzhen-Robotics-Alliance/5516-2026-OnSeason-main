package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbInputsAutoLogged inputs;

    public Climb(ClimbIO io) {
        this.io = io;
        io.setMotorOutput(0.0);
        inputs = new ClimbInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    /** Stops the climb motor output immediately. */
    public void stop() {
        io.setMotorOutput(0.0);
    }

    /**
     * Checks if the climb is at the upper limit position.
     *
     * @return true if at upper limit
     */
    public boolean isAtUpperLimit() {
        return inputs.climbAbsolutePosition <= UPPER_LIMIT_ROTATIONS;
    }

    /**
     * Checks if the climb is at the lower limit position.
     *
     * @return true if at lower limit
     */
    public boolean isAtLowerLimit() {
        return inputs.climbAbsolutePosition >= LOWER_LIMIT_ROTATIONS;
    }

    // Pov Up - move climb upward (decreasing position)
    public Command manualUpCommand() {
        return run(() -> {
                    if (!inputs.hardwareConnected || !inputs.absoluteEncoderConnected) {
                        io.setMotorOutput(0.0);
                        return;
                    }

                    // Check upper limit
                    if (isAtUpperLimit()) {
                        io.setMotorOutput(0.0);
                        return;
                    }

                    io.setMotorOutput(MANUAL_UP_VOLTAGE.in(Volts));
                })
                .finallyDo(this::stop);
    }

    // Pov Down - move climb downward (increasing position)
    public Command manualDownCommand() {
        return run(() -> {
                    if (!inputs.hardwareConnected || !inputs.absoluteEncoderConnected) {
                        io.setMotorOutput(0.0);
                        return;
                    }

                    // Check lower limit
                    if (isAtLowerLimit()) {
                        io.setMotorOutput(0.0);
                        return;
                    }

                    io.setMotorOutput(MANUAL_DOWN_VOLTAGE.in(Volts));
                })
                .finallyDo(this::stop);
    }
}
