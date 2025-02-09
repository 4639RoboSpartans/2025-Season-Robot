package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

import java.util.Objects;

@Deprecated
public abstract class AlgaeIntakeSubsystem extends SubsystemBase {
    private static AlgaeIntakeSubsystem instance;

    public static AlgaeIntakeSubsystem getInstance() {
        if (Robot.isReal()) {
            return instance = Objects.requireNonNullElseGet(instance, ConcreteAlgaeIntakeSubsystem::new);
        } else {
            return instance = Objects.requireNonNullElseGet(instance, SimAlgaeIntakeSubsystem::new);
        }
    }

    protected abstract void setIntakeState(AlgaeIntakeState intakeState);

    public Command setIntakeStateCommand(AlgaeIntakeState state) {
        return Commands.runOnce(
            () -> setIntakeState(state)
        );
    }

    public abstract Trigger atRequestedState();
}
