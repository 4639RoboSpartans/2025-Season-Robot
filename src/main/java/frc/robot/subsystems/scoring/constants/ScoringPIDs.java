package frc.robot.subsystems.scoring.constants;

import frc.lib.TunableNumber;

public class ScoringPIDs {
    public static TunableNumber elevatorKp = new TunableNumber("Concrete/Elevator kP").withDefaultValue(0);
    public static TunableNumber elevatorKi = new TunableNumber("Concrete/Elevator kI").withDefaultValue(0.0);
    public static TunableNumber elevatorKd = new TunableNumber("Elevator kD").withDefaultValue(0.0);
    public static TunableNumber elevatorVelocity = new TunableNumber("Concrete/Elevator Velocity").withDefaultValue(30);
    public static TunableNumber elevatorAcceleration = new TunableNumber("Concrete/Elevator Acceleration").withDefaultValue(10.0);
    public static TunableNumber elevatorKs = new TunableNumber("Concrete/Elevator Ks").withDefaultValue(0.0);
    public static TunableNumber elevatorKg = new TunableNumber("Concrete/Elevator Kg").withDefaultValue(0.63195475);
    public static TunableNumber elevatorKv = new TunableNumber("Concrete/Elevator Kv").withDefaultValue(0.085);
    public static TunableNumber elevatorKa = new TunableNumber("Concrete/Elevator Ka").withDefaultValue(0.0);

    public static TunableNumber wristKp = new TunableNumber("Concrete/Wrist kP").withDefaultValue(100.0);
    public static TunableNumber wristKi = new TunableNumber("Concrete/Wrist kI").withDefaultValue(0.0);
    public static TunableNumber wristKd = new TunableNumber("Concrete/Wrist kD").withDefaultValue(2.0);
    public static TunableNumber wristVelocity = new TunableNumber("Concrete/Wrist Velocity").withDefaultValue(30.0);
    public static TunableNumber wristAcceleration = new TunableNumber("Concrete/Wrist Acceleration").withDefaultValue(20.0);
    public static TunableNumber wristKs = new TunableNumber("Concrete/Wrist Ks").withDefaultValue(0.0);
    public static TunableNumber wristKg = new TunableNumber("Concrete/Wrist Kg").withDefaultValue(0.0);
    public static TunableNumber wristKv = new TunableNumber("Concrete/Wrist Kv").withDefaultValue(0.0);
    public static TunableNumber wristKa = new TunableNumber("Concrete/Wrist Ka").withDefaultValue(0.0);

    public static TunableNumber simElevatorKp = new TunableNumber("Sim/Elevator kP").withDefaultValue(0);
    public static TunableNumber simElevatorKi = new TunableNumber("Sim/Elevator kI").withDefaultValue(0.0);
    public static TunableNumber simElevatorKd = new TunableNumber("Sim/Elevator kD").withDefaultValue(0.0);
    public static TunableNumber simElevatorVelocity = new TunableNumber("Sim/Elevator Velocity").withDefaultValue(30);
    public static TunableNumber simElevatorAcceleration = new TunableNumber("Sim/Elevator Acceleration").withDefaultValue(10.0);
    public static TunableNumber simElevatorKs = new TunableNumber("Sim/Elevator Ks").withDefaultValue(0.0);
    public static TunableNumber simElevatorKg = new TunableNumber("Sim/Elevator Kg").withDefaultValue(0.63195475);
    public static TunableNumber simElevatorKv = new TunableNumber("Sim/Elevator Kv").withDefaultValue(0.085);
    public static TunableNumber simElevatorKa = new TunableNumber("Sim/Elevator Ka").withDefaultValue(0.0);

    public static TunableNumber simWristKp = new TunableNumber("Sim/Wrist kP").withDefaultValue(100.0);
    public static TunableNumber simWristKi = new TunableNumber("Sim/Wrist kI").withDefaultValue(0.0);
    public static TunableNumber simWristKd = new TunableNumber("Sim/Wrist kD").withDefaultValue(2.0);
    public static TunableNumber simWristVelocity = new TunableNumber("Sim/Wrist Velocity").withDefaultValue(30.0);
    public static TunableNumber simWristAcceleration = new TunableNumber("Sim/Wrist Acceleration").withDefaultValue(20.0);
    public static TunableNumber simWristKs = new TunableNumber("Sim/Wrist Ks").withDefaultValue(0.0);
    public static TunableNumber simWristKg = new TunableNumber("Sim/Wrist Kg").withDefaultValue(0.0);
    public static TunableNumber simWristKv = new TunableNumber("Sim/Wrist Kv").withDefaultValue(0.0);
    public static TunableNumber simWristKa = new TunableNumber("Sim/Wrist Ka").withDefaultValue(0.0);
}
