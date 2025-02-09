package frc.robot.subsystems.algaeIntake;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.algaeIntake.constants.AlgaeIntakeConstants.*;

@Deprecated
public enum AlgaeIntakeState {
    INTAKING(0, 0.5),
    OUTTAKING(0, -0.5),
    EXTENDED(0, 0),
    IDLE(1, 0);

    private final double positionPercentage;
    public final double intakeSpeed;

    AlgaeIntakeState(double positionPercentage, double intakeSpeed) {
        this.positionPercentage = positionPercentage;
        this.intakeSpeed = intakeSpeed;
    }

    public double getAbsolutePosition() {
        return PivotConstants.downPosition + positionPercentage * PivotConstants.positionDiff;
    }

    public Rotation2d getSimAngle() {
        return Rotation2d.fromDegrees(positionPercentage * 90);
    }
}
