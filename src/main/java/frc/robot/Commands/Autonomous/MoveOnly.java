// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autonomous;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveOnly extends SequentialCommandGroup {

  private final CommandSwerveDrivetrain m_SwerveSubsystem;
  private final SwerveRequest.FieldCentric m_drive;

  /** Creates a new WeekZeroAuto. */
  public MoveOnly(CommandSwerveDrivetrain in_SwerveSubsystem, SwerveRequest.FieldCentric drive) {

    m_SwerveSubsystem = in_SwerveSubsystem;
    m_drive = drive;
    
    addCommands(new ParallelCommandGroup(
        new RunCommand(() -> m_SwerveSubsystem.goForward(m_drive), m_SwerveSubsystem)
            .withTimeout(2)
            .andThen(new InstantCommand(() -> m_SwerveSubsystem.stop(m_drive), m_SwerveSubsystem))));

  }
}
