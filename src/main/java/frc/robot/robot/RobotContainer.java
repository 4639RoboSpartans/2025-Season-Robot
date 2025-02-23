// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FunctionalTrigger;
import frc.lib.led.LEDStrip;
import frc.lib.led.PhysicalLEDStrip;
import frc.robot.commands.AutoRoutines;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.SwerveAutoRoutinesCreator;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

import java.util.Arrays;

import static edu.wpi.first.units.Units.Meters;


public class RobotContainer {
    private final AbstractSwerveDrivetrain swerve = SubsystemManager.getInstance().getDrivetrain();
    private final ScoringSuperstructure scoringSuperstructure = SubsystemManager.getInstance().getScoringSuperstructure();
    @SuppressWarnings("unused")
    private final RobotSim robotSim = new RobotSim();
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Pose2d> startPositionChooser = new SendableChooser<>();
    private final LEDStrip strip = new PhysicalLEDStrip(9, 10);

    private final StructArrayPublisher<Pose3d> componentPoses = NetworkTableInstance.getDefault()
        .getStructArrayTopic("zeroed component poses", Pose3d.struct).publish();

    public RobotContainer() {

        // create auto routines here because we're configuring AutoBuilder in this method
        //TODO: take this out when we correctly refactor configureAutoBuilder to a new place
        AutoRoutines swerveAutoRoutines = SwerveAutoRoutinesCreator.createAutoRoutines(swerve);

        configureBindings();

        autoChooser = new SendableChooser<>();
        addAllCompAutons(autoChooser, swerveAutoRoutines);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        startPositionChooser.setDefaultOption("DEFAULT", new Pose2d());
        SmartDashboard.putBoolean("pigeon reset", false);
        Arrays.stream(FieldConstants.AutonStartingPositions.values()).forEach(
            position -> startPositionChooser.addOption(position.name(), position.Pose)
        );
        SmartDashboard.putData("Selected Reset Position", startPositionChooser);
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.manualControl());
        scoringSuperstructure.setDefaultCommand(scoringSuperstructure.runScoringState());

        //Scoring Controls
        {
            Controls.Driver.rotationResetTrigger.onTrue(
                swerve.resetPigeon()
            );

            Controls.Driver.BargeScoringTrigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_BARGE
                )
            );
            Controls.Operator.HPLoadingTrigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.INTAKE_FROM_HP
                )
            );
            Controls.Driver.ProcessorTrigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_PROCESSOR
                )
            );
            Controls.Operator.L1Trigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_L1_CORAL
                )
            );
            Controls.Operator.L2Trigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_L2_CORAL
                )
            );
            Controls.Operator.L3Trigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_L3_CORAL
                )
            );
            Controls.Operator.L4Trigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_L4_CORAL
                )
            );
            Controls.Driver.L2AlgaeTrigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.INTAKE_L2_ALGAE
                )
            );
            Controls.Driver.L3AlgaeTrigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.INTAKE_L3_ALGAE
                )
            );
            Controls.Operator.HoldTrigger.onTrue(
                scoringSuperstructure.hold()
            );

            Controls.Operator.ToggleManualControlTrigger.whileTrue(
                scoringSuperstructure.toggleManualControl()
            );
        }

        FunctionalTrigger.of(Controls.Driver.reefAlign)
                .and(Controls.Driver.targetLeft)
                .whileTrue(() -> DriveCommands.moveToClosestReefPosition((byte) 0));
        FunctionalTrigger.of(Controls.Driver.reefAlign)
                .and(Controls.Driver.targetRight)
                .whileTrue(() -> DriveCommands.moveToClosestReefPosition((byte) 1));
        FunctionalTrigger.of(Controls.Driver.reefAlign)
                .and(Controls.Driver.targetLeft.negate())
                .and(Controls.Driver.targetRight.negate())
                .whileTrue(() -> DriveCommands.moveToClosestReefPosition((byte) 2));

        FunctionalTrigger.of(Controls.Driver.coralStationAlign)
                .and(Controls.Driver.targetLeft).whileTrue(() -> DriveCommands.moveToDesiredCoralStationPosition(true));
        FunctionalTrigger.of(Controls.Driver.coralStationAlign)
                .and(Controls.Driver.targetRight).whileTrue(() -> DriveCommands.moveToDesiredCoralStationPosition(false));


        // OI.getInstance().operatorController().Y_BUTTON.whileTrue(
        //         ElevatorSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        // );
        // OI.getInstance().operatorController().A_BUTTON.whileTrue(
        //         ElevatorSysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        // );
        // OI.getInstance().operatorController().POV_UP.whileTrue(
        //         ElevatorSysID.sysIdDynamic(SysIdRoutine.Direction.kForward)
        // );
        // OI.getInstance().operatorController().POV_DOWN.whileTrue(
        //         ElevatorSysID.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        // );

        // OI.getInstance().driverController().A_BUTTON.onTrue(MiscellaneousCommands.ElevatorUpDownTest());
    }

    private void addAllCompAutons(SendableChooser<Command> autoChooser, AutoRoutines swerveAutoRoutines) {
        for (AutoRoutine a : swerveAutoRoutines.getAllCompRoutines()) {
            autoChooser.addOption(a.toString(), a.cmd());
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void add3DComponentPoses() {
        componentPoses.set(
            new Pose3d[]{
                new Pose3d(
                    new Translation3d(
                        0,
                        0,
                        (scoringSuperstructure.getCurrentElevatorLength().in(Meters)
                            - ScoringConstants.ElevatorConstants.STARTING_HEIGHT.in(Meters)) / 3.0),
                    new Rotation3d()
                ),
                new Pose3d(
                    new Translation3d(
                        0,
                        0,
                        (scoringSuperstructure.getCurrentElevatorLength().in(Meters)
                            - ScoringConstants.ElevatorConstants.STARTING_HEIGHT.in(Meters)) * 2.0 / 3.0),
                    new Rotation3d()
                ),
                new Pose3d(
                    new Translation3d(
                        0,
                        0,
                        (scoringSuperstructure.getCurrentElevatorLength().in(Meters)
                            - ScoringConstants.ElevatorConstants.STARTING_HEIGHT.in(Meters))),
                    new Rotation3d()
                ),
                new Pose3d(
                    new Translation3d(
                        0,
                        0,
                        (scoringSuperstructure.getCurrentElevatorLength().in(Meters)
                            - ScoringConstants.ElevatorConstants.STARTING_HEIGHT.in(Meters))
                    ).plus(
                        ScoringConstants.EndEffectorConstants.Hopper3DSimOffset
                    ),
                    new Rotation3d(
                        0,
                        -scoringSuperstructure.getCurrentWristRotation().minus(ScoringConstants.EndEffectorConstants.IDLE_ROTATION).getRadians(),
                        0
                    )
                )
            }
        );
    }
}
