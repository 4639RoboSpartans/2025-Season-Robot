package frc.robot.subsystems.coralIntake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coralIntake.constants.AlgaeIntakeConstants;
import frc.robot.subsystems.coralIntake.constants.AlgaeIntakePIDs;

public class ConcreteAlgaeIntakeSubsystem extends AlgaeIntakeSubsystem implements Subsystem {
    private final TalonFX intakePivot;
    private final SparkFlex intakeMotor;

    private final MotionMagicVoltage pivotControlRequest;

    private AlgaeIntakeState intakeState = AlgaeIntakeState.IDLE;

    public ConcreteAlgaeIntakeSubsystem() {
        intakePivot = new TalonFX(AlgaeIntakeConstants.IDs.pivotID);
        TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(
                                        AlgaeIntakePIDs.pivotVelocity.get()
                                )
                                .withMotionMagicAcceleration(
                                        AlgaeIntakePIDs.pivotAcceleration.get()
                                )
                )
                .withSlot0(
                        new Slot0Configs()
                                .withKP(
                                        AlgaeIntakePIDs.pivotKp.get()
                                )
                                .withKI(
                                        AlgaeIntakePIDs.pivotKi.get()
                                )
                                .withKD(
                                        AlgaeIntakePIDs.pivotKd.get()
                                )
                );
        intakePivot.getConfigurator().apply(config);
        intakeMotor = new SparkFlex(
                AlgaeIntakeConstants.IDs.intakeID,
                SparkLowLevel.MotorType.kBrushless
        );
        pivotControlRequest = new MotionMagicVoltage(intakeState.getAbsolutePosition());
    }

    protected void setIntakeState(AlgaeIntakeState intakeState) {
        this.intakeState = intakeState;
        intakeMotor.set(intakeState.intakeSpeed);
        pivotControlRequest.Position = intakeState.getAbsolutePosition();
        intakePivot.setControl(pivotControlRequest);
    }

    public Trigger atRequestedState() {
        return new Trigger(
                () -> MathUtil.isNear(
                        pivotControlRequest.Position,
                        intakePivot.getPosition(true).getValueAsDouble(),
                        AlgaeIntakeConstants.PivotConstants.positionTolerance
                )
        );
    }
}
