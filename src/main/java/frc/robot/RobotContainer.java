// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Autonomous.AutoSequences;
import frc.robot.Commands.Autonomous.MoveOnly;
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
import com.pathplanner.lib.path.PathPlannerPath;


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
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private SendableChooser<Command> trajChooser;

  public RobotContainer() {

    // drivetrain.getPigeon2().setYaw(-90);
    // drivetrain.tareEverything();

    AutoSequences autoSeq = new AutoSequences(pivot, intake, firingHead, masterController);

    NamedCommands.registerCommand("SetShooterSpeed", new InstantCommand(() -> firingHead.shooterSetSpeed(Constants.FiringHeadConstants.NearFiringSpeed),firingHead));
    // new WaitCommand(0.55),
    
    NamedCommands.registerCommand("TransportOn",  new InstantCommand( () -> firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.TransportMotorSpeed), firingHead));
    NamedCommands.registerCommand("ShootTransportOn",  new InstantCommand( () -> firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), firingHead));
    // new WaitCommand(1),
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
    NamedCommands.registerCommand("GoToStarting", new InstantCommand(() -> pivot.GoToStarting(), pivot));

    NamedCommands.registerCommand("starting_sequence", autoSeq.StartingSequence());
    NamedCommands.registerCommand("run_conveyors", autoSeq.ConveyorSequence());
    NamedCommands.registerCommand("run_conveyors_until", autoSeq.ConveyorSequenceUntilSensor());
    NamedCommands.registerCommand("stop_conveyors", autoSeq.StopSequence());

    NamedCommands.registerCommand("fire_dump", autoSeq.ShootingSequence_Dump());
    NamedCommands.registerCommand("fire_near", autoSeq.ShootingSequence_Near());
    NamedCommands.registerCommand("fire_far", autoSeq.ShootingSequence_Far());

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

    // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driverController.b().whileTrue(drivetrain
    //     .applyRequest(() -> point
    //         .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
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

    // driverController.y().onTrue(new InstantCommand(() -> pivot.up(), pivot));
    // driverController.a().onTrue(new InstantCommand(() -> pivot.down(), pivot));

    driverController.rightTrigger().onTrue(new InstantCommand(() -> firingHead.shooterSetSpeed(masterController.getFiringSpeed()), firingHead)
    .andThen(new WaitCommand(.2))
    .andThen(new InstantCommand(() -> firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.ShootTransportMotorSpeed), firingHead))
    .andThen(new WaitCommand(3))
    .andThen(new InstantCommand(() -> firingHead.MasterStop(), firingHead)));
    //.andThen(new InstantCommand(() -> pivot.GoToStarting(), pivot)));      

    // driverController.rightBumper().onTrue(new RunCommand(() -> masterController.runConveyors(), masterController)
    // .until(() -> (firingHead.CenterSensorTriggered() && pivot.InStartingPosition())
    //     || (intake.EitherSensorTriggered() && !pivot.InStartingPosition())
    //     || operatorController.leftTrigger().getAsBoolean()
    //     || operatorController.start().getAsBoolean()
    //     || driverController.start().getAsBoolean())
    // .andThen(new InstantCommand(() -> masterController.stopConveyors(), masterController)));  

    driverController.start().onTrue(home);

    // ********************* operator control **************************/
    operatorController.x().whileTrue(new InstantCommand(() -> lift.up(), lift));
    operatorController.x().onFalse(new InstantCommand(() -> lift.stop(), lift));

    operatorController.b().whileTrue(new InstantCommand(() -> lift.down(), lift));
    operatorController.b().onFalse(new InstantCommand(() -> lift.stop(), lift));

    operatorController.start().onTrue(home); // pivot starting position

    operatorController.y().onTrue(new InstantCommand(() -> pivot.up(), pivot));
    operatorController.a().onTrue(new InstantCommand(() -> pivot.down(), pivot));

    //TODO: make head go to start on intake on
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
    // return new WeekZeroAuto(pivot, intake, firingHead, masterController, drivetrain, drive);
    //return new MoveOnly(drivetrain, drive);
    // return Commands.sequence(
    //   new InstantCommand(()->drivetrain.seedFieldRelative(new Pose2d(1, 1, new Rotation2d(0))))
      // ,
      // drivetrain.applyRequest(() -> drive.withVelocityX(MaxSpeed) // Drive forward with
      //       // negative Y (forward)
      //       .withVelocityY(0) // Drive left with negative X (left)
      //       .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      //   )
        // );
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
