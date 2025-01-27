package frc.robot.subsystems.algaeIntake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.algaeIntake.constants.AlgaeIntakeConstants;
import frc.robot.subsystems.algaeIntake.constants.AlgaeIntakePIDs;

public class ConcreteAlgaeIntakeSubsystem extends AlgaeIntakeSubsystem implements Subsystem {
    private final SparkMax intakePivot;
    private final SparkFlex intakeMotor;

    private final DutyCycleEncoder pivotEncoder;

    private final ProfiledPIDController pivotPID;

    private AlgaeIntakeState intakeState = AlgaeIntakeState.IDLE;

    public ConcreteAlgaeIntakeSubsystem() {
        intakePivot = new SparkMax(
                AlgaeIntakeConstants.IDs.pivotID,
                SparkLowLevel.MotorType.kBrushless
        );
        intakePivot.configure(
                new SparkMaxConfig()
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(40)
                                        .reverseSoftLimit(40)
                        ),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );
        intakeMotor = new SparkFlex(
                AlgaeIntakeConstants.IDs.intakeID,
                SparkLowLevel.MotorType.kBrushless
        );
        pivotEncoder = new DutyCycleEncoder(AlgaeIntakeConstants.IDs.encoderID);
        pivotPID = new ProfiledPIDController(
                AlgaeIntakePIDs.pivotKp.get(),
                AlgaeIntakePIDs.pivotKi.get(),
                AlgaeIntakePIDs.pivotKd.get(),
                new TrapezoidProfile.Constraints(
                        AlgaeIntakePIDs.pivotVelocity.get(),
                        AlgaeIntakePIDs.pivotAcceleration.get()
                )
        );
    }

    protected void setIntakeState(AlgaeIntakeState intakeState) {
        this.intakeState = intakeState;
        intakeMotor.set(intakeState.intakeSpeed);
        pivotPID.setGoal(intakeState.getAbsolutePosition());
    }

    public Trigger atRequestedState() {
        return new Trigger(
                () -> MathUtil.isNear(
                        pivotPID.getGoal().position,
                        pivotEncoder.get(),
                        AlgaeIntakeConstants.PivotConstants.positionTolerance
                )
        );
    }

    @Override
    public void periodic() {
        updatePIDs();
        intakePivot.setVoltage(
                pivotPID.calculate(pivotEncoder.get())
        );
    }

    private void updatePIDs() {
        pivotPID.setPID(
                AlgaeIntakePIDs.pivotKp.get(),
                AlgaeIntakePIDs.pivotKi.get(),
                AlgaeIntakePIDs.pivotKd.get()
        );
        pivotPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        AlgaeIntakePIDs.pivotVelocity.get(),
                        AlgaeIntakePIDs.pivotAcceleration.get()
                )
        );
    }
}
