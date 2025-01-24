package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Objects;

public abstract class AlgaeIntakeSubsystem extends SubsystemBase {
    private static AlgaeIntakeSubsystem instance;

    public static AlgaeIntakeSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, ConcreteAlgaeIntakeSubsystem::new);
    }

    protected abstract void setIntakeState(AlgaeIntakeState intakeState);

    public Command setIntakeStateCommand(AlgaeIntakeState state) {
        return Commands.runOnce(
            () -> setIntakeState(state)
        );
    }

    public abstract Trigger atRequestedState();
}
