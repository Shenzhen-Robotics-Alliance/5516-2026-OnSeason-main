package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbInputsAutoLogged inputs;
    private boolean climbReady = false;

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

    public Command climbCommand() {
        return run(() -> io.setMotorOutput(-6.5))
                .until(() -> inputs.climbAbsolutePosition <= 0.05)
                .until(() -> !inputs.hardwareConnected)
                .finallyDo(() -> io.setMotorOutput(0.0))
                .onlyIf(() -> climbReady);
    }

    public Command prepareClimbCommand() {
        return Commands.sequence(
                        run(() -> io.setMotorOutput(10.0)).until(() -> inputs.climbAbsolutePosition >= 0.29),
                        Commands.runOnce(() -> climbReady = true))
                .until(() -> !inputs.hardwareConnected)
                .finallyDo(() -> {
                    io.setMotorOutput(0.0);
                    io.setSupportMotorOutput(0.0);
                })
                .alongWith(run(() -> io.setSupportMotorOutput(6.5))
                        .withTimeout(1.0)
                        .finallyDo(() -> io.setSupportMotorOutput(0.0)));
    }

    public Command cancelClimb() {
        return Commands.sequence(climbCommand())
                .finallyDo(() -> climbReady = false)
                .alongWith(run(() -> io.setSupportMotorOutput(-6.5))
                        .withTimeout(1.0)
                        .finallyDo(() -> io.setSupportMotorOutput(0.0)));
    }
}
