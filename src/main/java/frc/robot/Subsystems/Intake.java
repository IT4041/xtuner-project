// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private CANSparkMax intake;
  private CANSparkMax conveyrUp;
  private CANSparkMax conveyrLow;

  enum Stages {
    Idle,
    Triggered,
    Stopped,
    Post
  }

  Stages stage = Stages.Idle;

  private final TimeOfFlight SideSensor = new TimeOfFlight(Constants.IntakeConstants.TimeOfFlightSideSensorID);
  private final TimeOfFlight topSensor = new TimeOfFlight(Constants.IntakeConstants.TimeOfFlightTopSensorID);

  public Intake() {

    // lower intake
    intake = new CANSparkMax(Constants.IntakeConstants.LowerIntakeSparkmaxDeviceID, MotorType.kBrushless);
    intake.restoreFactoryDefaults();
    intake.setIdleMode(IdleMode.kBrake);
    intake.setSmartCurrentLimit(80);
    intake.setClosedLoopRampRate(1);

    // upper conveyor
    conveyrUp = new CANSparkMax(Constants.IntakeConstants.UpperConvyerSparkmaxDeviceID, MotorType.kBrushless);
    conveyrUp.restoreFactoryDefaults();
    conveyrUp.setIdleMode(IdleMode.kBrake);
    conveyrUp.setSmartCurrentLimit(80);
    conveyrUp.setClosedLoopRampRate(1);
    conveyrUp.setInverted(true);

    // lower conveyor
    conveyrLow = new CANSparkMax(Constants.IntakeConstants.LowerConvyerSparkmaxDeviceID, MotorType.kBrushless);
    conveyrLow.restoreFactoryDefaults();
    conveyrLow.setIdleMode(IdleMode.kBrake);
    conveyrLow.setSmartCurrentLimit(80);
    conveyrLow.setClosedLoopRampRate(1);

    SideSensor.setRangingMode(RangingMode.Medium, 1);
    topSensor.setRangingMode(RangingMode.Long, 1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Intake Stage", stage.toString());
    SmartDashboard.putBoolean("Intake A triggered?", this.SideTriggered());
    SmartDashboard.putBoolean("Intake B triggered?", this.TopTriggered());
    SmartDashboard.putNumber("Intake A distance", SideSensor.getRange());
    SmartDashboard.putNumber("Intake b distance", topSensor.getRange());
    SmartDashboard.putBoolean("intake head is on", intake.get() > 0);
  }

  public boolean SideTriggered() {
    return SideSensor.getRange() <= Constants.IntakeConstants.SideTreshholdIntake;
  }

  public boolean TopTriggered() {
    return topSensor.getRange() <= Constants.IntakeConstants.TopTreshholdIntake;
  }

  public boolean EitherSensorTriggered() {
    return this.SideTriggered(); //|| this.TopTriggered();
  }

  public void on() {

    conveyrLow.set(Constants.IntakeConstants.ConveyrMotorSpeed);
    conveyrUp.set(-Constants.IntakeConstants.ConveyrMotorSpeed);
    intake.set(Constants.IntakeConstants.IntakeMotorSpeed);
    
    stage = Stages.Triggered;
  }

  public void off() {
    intake.stopMotor();
    conveyrLow.stopMotor();
    conveyrUp.stopMotor();

    stage = Stages.Idle;
  }

  public void setIntakeSpeed(double in_speed){
    intake.set(in_speed); 
  }

  public void setConveyorSpeed(double con_speed){
    conveyrUp.set(con_speed);
    conveyrLow.set(con_speed);
  }

  public void TempTrig() {
    if (stage==Stages.Idle) {
      stage=Stages.Triggered;
      setIntakeSpeed(Constants.IntakeConstants.ConveyrMotorSpeed);
      setConveyorSpeed(Constants.IntakeConstants.ConveyrMotorSpeed);
      
    } else { if(stage==Stages.Triggered){
        stage = Stages.Idle;
        setIntakeSpeed(0);
        setConveyorSpeed(0);
      }
    }
  }
}
