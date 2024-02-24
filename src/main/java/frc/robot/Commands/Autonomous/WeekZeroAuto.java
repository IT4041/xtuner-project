// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WeekZeroAuto extends SequentialCommandGroup {
  private final Intake m_intake;
  private final Pivot m_pivot;
  private final FiringHead m_firingHead;
  private final MasterController m_masterController;
  private final CommandSwerveDrivetrain m_SwerveSubsystem;

  /** Creates a new WeekZeroAuto. */
  public WeekZeroAuto(Pivot in_pivot, Intake in_intake, FiringHead in_firingHead, MasterController in_masterController,
      CommandSwerveDrivetrain in_SwerveSubsystem) {
    m_intake = in_intake;
    m_pivot = in_pivot;
    m_firingHead = in_firingHead;
    m_masterController = in_masterController;
    m_SwerveSubsystem = in_SwerveSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new AutoSequences(m_pivot, m_intake, m_firingHead, m_masterController).AutoStartingSequence());

    addCommands(new ParallelCommandGroup(
        new RunCommand(() -> m_SwerveSubsystem.goForward(-0.4), m_SwerveSubsystem)
            .withTimeout(3.35)
            .andThen(new InstantCommand(() -> m_SwerveSubsystem.stop(), m_SwerveSubsystem)),
        new RunCommand(() -> m_masterController.runConveyors(), m_masterController)
            .until(() -> m_firingHead.EitherSensorTriggered())
            .andThen(new InstantCommand(() -> m_masterController.stopConveyors(), m_masterController))));

    addCommands(new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FarFiringSpeed),
        m_firingHead));
    addCommands(new InstantCommand(
        () -> m_pivot.GoToShootingMidRange(), m_pivot)); // 37
    addCommands(new WaitCommand(0.5));
    addCommands(new InstantCommand(
        () -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.TransportMotorSpeed), m_firingHead));
    addCommands(new WaitCommand(2));

    addCommands(new ParallelCommandGroup(
        new InstantCommand(() -> m_firingHead.setTransportMotorSpeed(0), m_firingHead), // shooter
        new InstantCommand(() -> m_intake.setConveyorSpeed(0), m_intake) // intake
    ));

    addCommands(new ParallelCommandGroup(
        new InstantCommand(() -> m_firingHead.shooterSetSpeed(0), m_firingHead), // shooter
        new InstantCommand(() -> m_intake.setIntakeSpeed(0), m_intake) // intake
    ));

    addCommands(
        new InstantCommand(() -> m_pivot.GoToStarting(), m_pivot));
  }
}
