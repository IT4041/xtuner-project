// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Autonomous.AutoSequences;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.FiringHead;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.MasterController;
import frc.robot.Subsystems.Pivot;
import frc.robot.generated.TunerConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final CommandXboxController driverController = new CommandXboxController(
      Constants.kDriverControllerPort);
  public static final CommandXboxController operatorController = new CommandXboxController(
      Constants.kOperatorControllerPort);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Pivot pivot = new Pivot();
  private final Intake intake = new Intake();
  private final FiringHead firingHead = new FiringHead();
  private final Lift lift = new Lift();
  private final LED led = new LED(this);
  private final MasterController masterController = new MasterController(pivot, intake, firingHead, led, driverController, operatorController);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private SendableChooser<Command> trajChooser;

  public RobotContainer() {

    AutoSequences autoSeq = new AutoSequences(pivot, intake, firingHead, masterController);

    NamedCommands.registerCommand("SetShooterSpeed", new InstantCommand(() -> firingHead.shooterSetSpeed(Constants.FiringHeadConstants.NearFiringSpeed),firingHead));
    NamedCommands.registerCommand("TransportOn",  new InstantCommand( () -> firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.TransportMotorSpeed), firingHead));
    NamedCommands.registerCommand("ShootTransportOn",  new InstantCommand( () -> firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), firingHead));
    NamedCommands.registerCommand("ShooterOff", new InstantCommand(() -> firingHead.shooterSetSpeed(0d), firingHead));
    NamedCommands.registerCommand("PivotStarting", new InstantCommand(() -> pivot.GoToStarting(), pivot));
    NamedCommands.registerCommand("IntakeOn", new InstantCommand(() -> intake.setIntakeSpeed(Constants.IntakeConstants.IntakeMotorSpeed), intake));
    NamedCommands.registerCommand("ConveyorOn", new InstantCommand(() -> intake.setConveyorSpeed(Constants.IntakeConstants.ConveyrMotorSpeed), intake));
    NamedCommands.registerCommand("near_shooting", new InstantCommand(() -> firingHead.shooterSetSpeed(Constants.FiringHeadConstants.NearFiringSpeed), firingHead));
    NamedCommands.registerCommand("far_shooting", new InstantCommand(() -> firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FarFiringSpeed), firingHead));
    NamedCommands.registerCommand("dump_shooting", new InstantCommand(() -> firingHead.shooterSetSpeed(Constants.FiringHeadConstants.DumpSpeed), firingHead));
    NamedCommands.registerCommand("GoToDump", new InstantCommand(() -> pivot.GoToDump(), pivot));
    NamedCommands.registerCommand("GoToShootingMidRange", new InstantCommand(() -> pivot.GoToShootingMidRange(), pivot));
    NamedCommands.registerCommand("GoToShootingShortRange", new InstantCommand(() -> pivot.GoToShootingShortRange(), pivot));

    NamedCommands.registerCommand("GoToLeftNoteShootingPosition", new InstantCommand(() -> pivot.GoToLeftNoteShootingPosition(), pivot));
    NamedCommands.registerCommand("GoToRightNoteShootingPosition", new InstantCommand(() -> pivot.GoToRightNoteShootingPosition(), pivot));
    NamedCommands.registerCommand("GoToCenterNoteShootingPosition", new InstantCommand(() -> pivot.GoToCenterNoteShootingPosition(), pivot));

    NamedCommands.registerCommand("GoToStarting", new InstantCommand(() -> pivot.GoToStarting(), pivot));
    NamedCommands.registerCommand("starting_sequence", autoSeq.StartingSequence());
    NamedCommands.registerCommand("run_conveyors", autoSeq.ConveyorSequence());
    NamedCommands.registerCommand("run_conveyors_until", autoSeq.ConveyorSequenceUntilSensor());
    NamedCommands.registerCommand("stop_conveyors", autoSeq.StopSequence());
    NamedCommands.registerCommand("fire_dump", autoSeq.ShootingSequence_Dump());
    NamedCommands.registerCommand("fire_near", autoSeq.ShootingSequence_Near());
    NamedCommands.registerCommand("fire_far", autoSeq.ShootingSequence_Far());
    NamedCommands.registerCommand("fire_LeftNote", autoSeq.ShootingSequence_LeftNote());
    NamedCommands.registerCommand("fire_CenterNote", autoSeq.ShootingSequence_CenterNote());
    NamedCommands.registerCommand("fire_RightNote", autoSeq.ShootingSequence_RightNote());
    NamedCommands.registerCommand("fire_FarNote", autoSeq.ShootingSequence_FarNote());
    
    trajChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", trajChooser);
    configureBindings();
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
            // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // reset the field-centric heading on left bumper press
    driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);

    SequentialCommandGroup home = new SequentialCommandGroup(
        new InstantCommand(() -> firingHead.shooterSetSpeed(0), firingHead), // shooter off
        new InstantCommand(() -> intake.setIntakeSpeed(0), intake), // intake off
        new InstantCommand(() -> firingHead.setTransportMotorSpeed(0), firingHead), // transport motor off
        new InstantCommand(() -> intake.setConveyorSpeed(0), intake), // conveyr off
        new InstantCommand(() -> pivot.GoToStarting(), pivot) // pivot starting position
    );

    driverController.leftTrigger().onTrue(
        new InstantCommand(() -> firingHead.Source(), firingHead).repeatedly()
        .until(() -> firingHead.SideSensorTriggered())
        .andThen(new InstantCommand(() -> firingHead.MasterStop(), firingHead))
    );

    driverController.rightTrigger().onTrue(new InstantCommand(() -> firingHead.shooterSetSpeed(masterController.getFiringSpeed()), firingHead)
      .andThen(new WaitCommand(.1))
      .andThen(new InstantCommand(() -> firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), firingHead))
      .andThen(new WaitCommand(1))
      .andThen(new InstantCommand(() -> firingHead.MasterStop(), firingHead)));

    driverController.leftBumper().whileTrue(new InstantCommand(() -> masterController.OverrideOn(), masterController));
    driverController.leftBumper().onFalse(new InstantCommand(() -> masterController.OverrideOff(), masterController));

    driverController.start().onTrue(home);

    // ********************* operator control **************************/
    operatorController.x().whileTrue(new InstantCommand(() -> lift.up(), lift));
    operatorController.x().onFalse(new InstantCommand(() -> lift.stop(), lift));

    operatorController.b().whileTrue(new InstantCommand(() -> lift.down(), lift));
    operatorController.b().onFalse(new InstantCommand(() -> lift.stop(), lift));

    operatorController.start().onTrue(home); // pivot starting position

    operatorController.y().onTrue(new InstantCommand(() -> pivot.up(), pivot));
    operatorController.a().onTrue(new InstantCommand(() -> pivot.down(), pivot));

    operatorController.rightTrigger().onTrue(new RunCommand(() -> masterController.runConveyors(), masterController)
        .until(() -> (firingHead.CenterSensorTriggered() && pivot.InStartingPosition())
            || (intake.EitherSensorTriggered() && !pivot.InStartingPosition())
            || operatorController.leftTrigger().getAsBoolean()
            || operatorController.start().getAsBoolean()
            || driverController.start().getAsBoolean())
        .andThen(new InstantCommand(() -> masterController.stopConveyors(), masterController)));
  }

  public Command getAutonomousCommand() {
    return trajChooser.getSelected();
  }

  public void seedFieldRelative(){
    drivetrain.runOnce(() -> drivetrain.seedFieldRelative());
  }

public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return true;
  }

  public boolean isDisabled() {
    return DriverStation.isDisabled();
  }

}
