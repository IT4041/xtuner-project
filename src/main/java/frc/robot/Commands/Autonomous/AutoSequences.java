// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.FiringHead;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.MasterController;
import frc.robot.Subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

/** Add your docs here. */
public class AutoSequences {
    private final Intake m_intake;
    private final Pivot m_pivot;
    private final FiringHead m_firingHead;
    private final MasterController m_masterController;

    public AutoSequences(Pivot in_pivot, Intake in_intake, FiringHead in_firingHead, MasterController in_masterController) {
        m_intake = in_intake;
        m_pivot = in_pivot;
        m_firingHead = in_firingHead;
        m_masterController = in_masterController;
    }

    public SequentialCommandGroup ShootingSequence_Near() {

        SequentialCommandGroup shootingCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.NearFiringSpeed), m_firingHead),
                new InstantCommand(() -> m_pivot.GoToShootingShortRange(), m_pivot),
                new WaitCommand(.95),
                new InstantCommand( () -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), m_firingHead),
                new WaitCommand(.35));

        return shootingCommand;
    }

    public SequentialCommandGroup ShootingSequence_Far() {

        SequentialCommandGroup shootingCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FarFiringSpeed), m_firingHead),
                new InstantCommand(() -> m_pivot.GoToShootingMidRange(), m_pivot),
                new WaitCommand(.95),
                new InstantCommand( () -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), m_firingHead),
                new WaitCommand(.35));

        return shootingCommand;
    }

    public SequentialCommandGroup ShootingSequence_Right3Note() {

        SequentialCommandGroup shootingCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FarFiringSpeed), m_firingHead),
                new InstantCommand(() -> m_pivot.GoToRight3NoteShootingPosition(), m_pivot),
                new WaitCommand(.95),
                new InstantCommand( () -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), m_firingHead),
                new WaitCommand(.35));

        return shootingCommand;
    }

    public SequentialCommandGroup ShootingSequence_LeftNote() {

        SequentialCommandGroup shootingCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FarFiringSpeed), m_firingHead),
                new InstantCommand(() -> m_pivot.GoToLeftNoteShootingPosition(), m_pivot),
                new WaitCommand(.95),
                new InstantCommand( () -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), m_firingHead),
                new WaitCommand(.35));

        return shootingCommand;
    }

    public SequentialCommandGroup ShootingSequence_CenterNote() {

        SequentialCommandGroup shootingCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FarFiringSpeed), m_firingHead),
                new InstantCommand(() -> m_pivot.GoToCenterNoteShootingPosition(), m_pivot),
                new WaitCommand(.95),
                new InstantCommand( () -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), m_firingHead),
                new WaitCommand(.35));

        return shootingCommand;
    }

    public SequentialCommandGroup ShootingSequence_RightNote() {

        SequentialCommandGroup shootingCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FarFiringSpeed), m_firingHead),
                new InstantCommand(() -> m_pivot.GoToRightNoteShootingPosition(), m_pivot),
                new WaitCommand(.95),
                new InstantCommand( () -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), m_firingHead),
                new WaitCommand(.35));

        return shootingCommand;
    }

    public SequentialCommandGroup ShootingSequence_FarNote() {

        SequentialCommandGroup shootingCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FarFiringSpeed), m_firingHead),
                new InstantCommand(() -> m_pivot.GoToShootingMidRange(), m_pivot),
                new WaitCommand(.95),
                new InstantCommand( () -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), m_firingHead),
                new WaitCommand(.35));

        return shootingCommand;
    }

    public SequentialCommandGroup ShootingSequence_Dump() {

        SequentialCommandGroup shootingCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.DumpSpeed), m_firingHead),
                new InstantCommand(() -> m_pivot.GoToDump(), m_pivot),
                new WaitCommand(.8),
                new InstantCommand( () -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), m_firingHead),
                new WaitCommand(.35));

        return shootingCommand;
    }

    public SequentialCommandGroup StartingSequence() {

        SequentialCommandGroup group = new SequentialCommandGroup(
                this.ShootingSequence_Near(),
                new InstantCommand(() -> m_firingHead.shooterSetSpeed(0), m_firingHead),
                new InstantCommand(() -> m_pivot.GoToStarting(), m_pivot));
                //this.ConveyorSequenceUntilSensor());
                // new InstantCommand(() -> m_intake.setIntakeSpeed(Constants.IntakeConstants.IntakeMotorSpeed), m_intake),
                // new InstantCommand(() -> m_intake.setConveyorSpeed(Constants.IntakeConstants.ConveyrMotorSpeed), m_intake));

        return group;
    }

    public SequentialCommandGroup ConveyorSequence() {

        SequentialCommandGroup command = new RunCommand(() -> m_masterController.runConveyors(), m_masterController)
                .until(() -> m_firingHead.EitherSensorTriggered())
                .withTimeout(5)
                .andThen(new InstantCommand(() -> m_masterController.stopConveyors(), m_masterController));

        return command;
    }

    public SequentialCommandGroup ConveyorSequenceUntilSensor() {

        SequentialCommandGroup command = new RunCommand(() -> m_masterController.runConveyors(), m_masterController)
                .until(() -> m_firingHead.CenterSensorTriggered()).withTimeout(3)
                .andThen(new InstantCommand(() -> m_masterController.stopConveyors(), m_masterController));

        return command;
    }

    public SequentialCommandGroup StopSequence() {

        SequentialCommandGroup stopCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_firingHead.setTransportMotorSpeed(0), m_firingHead),
                new InstantCommand(() -> m_intake.setConveyorSpeed(0), m_intake),
                new InstantCommand(() -> m_firingHead.shooterSetSpeed(0), m_firingHead),
                new InstantCommand(() -> m_intake.setIntakeSpeed(0), m_intake));

        return stopCommand;
    }
}
