// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.oi.OI;
import frc.robot.commands.AutoRoutines;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Arrays;

import choreo.auto.AutoRoutine;


public class RobotContainer {
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Pose2d> m_startPositionChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Auto 1", swerve.getAutoRoutines().auto1().cmd());
        autoChooser.addOption("Auto 2", swerve.getAutoRoutines().auto2().cmd());
        autoChooser.addOption("Auto 3", swerve.getAutoRoutines().auto3().cmd());
        autoChooser.addOption("Auto 4", swerve.getAutoRoutines().auto4().cmd());
        SmartDashboard.putData("Auto Chooser", autoChooser);

        m_startPositionChooser.setDefaultOption("DEFAULT", new Pose2d());
        SmartDashboard.putBoolean("pigeon reset", false);
        Arrays.stream(FieldConstants.AutonStartingPositions.values()).forEach(
                position -> m_startPositionChooser.addOption(position.name(), position.Pose)
        );
        SmartDashboard.putData("Selected Reset Position", m_startPositionChooser);
    }

    private void configureBindings() {
        /*OI.getInstance().driverController().A_BUTTON.whileTrue(
            DriveSysID.sysIdQuasistatic(Direction.kForward)
        );
        OI.getInstance().driverController().B_BUTTON.whileTrue(
                DriveSysID.sysIdQuasistatic(Direction.kReverse)
        );
        OI.getInstance().driverController().X_BUTTON.whileTrue(
                DriveSysID.sysIdDynamic(Direction.kForward)
        );
        OI.getInstance().driverController().Y_BUTTON.whileTrue(
                DriveSysID.sysIdDynamic(Direction.kReverse)
        );*/
        swerve.setDefaultCommand(swerve.applyRequest(swerve::fieldCentricRequestSupplier));
        Controls.Driver.rotationResetTrigger.onTrue(
            swerve.resetPigeonCommand()
        )
        .onTrue(
            new RunCommand(
                () -> SmartDashboard.putBoolean("pigeon reset", true)
            )
        )
        .onFalse(
            new RunCommand(
                () -> SmartDashboard.putBoolean("pigeon reset", false)        
            )
        );

        /** Resets Pose to desired pose set by dashboard */
        OI.getInstance().driverController().RIGHT_STICK.whileTrue(
            swerve.run(() -> 
                swerve.resetPose(
                    m_startPositionChooser.getSelected()
                )
            )
        );

        VisionSubsystem.getInstance().setDefaultCommand(VisionSubsystem.getInstance().globalVision());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
