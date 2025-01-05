package frc.robot.subsystems.climber;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;

    public static ClimberSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, ConcreteClimberSubsystem::new);
    }
}