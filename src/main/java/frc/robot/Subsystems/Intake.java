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
  private CANSparkMax conveyrUpFollower;
  private CANSparkMax conveyrLowLeader;

  private double[] sideAccumulator = new double[20];
  private int sideAccumulatorIndex = 0;

  private final TimeOfFlight sideSensor = new TimeOfFlight(Constants.IntakeConstants.TimeOfFlightSideSensorID);

  public Intake() {

    // intake
    intake = new CANSparkMax(Constants.IntakeConstants.IntakeSparkmaxDeviceID, MotorType.kBrushless);// id:11
    intake.restoreFactoryDefaults();
    intake.setIdleMode(IdleMode.kBrake);
    intake.setSmartCurrentLimit(80);
    intake.setClosedLoopRampRate(1);

    // lower conveyor and upper conveyor
    conveyrLowLeader = new CANSparkMax(Constants.IntakeConstants.LowerConvyerSparkmaxDeviceID, MotorType.kBrushless);//id:2
    conveyrUpFollower = new CANSparkMax(Constants.IntakeConstants.UpperConvyerSparkmaxDeviceID, MotorType.kBrushless);//id:1

    conveyrLowLeader.restoreFactoryDefaults();
    conveyrUpFollower.restoreFactoryDefaults();

    conveyrLowLeader.setIdleMode(IdleMode.kBrake);
    conveyrUpFollower.setIdleMode(IdleMode.kBrake);

    conveyrLowLeader.setSmartCurrentLimit(80);
    conveyrUpFollower.setSmartCurrentLimit(80);

    conveyrLowLeader.enableVoltageCompensation(12);
    conveyrUpFollower.enableVoltageCompensation(12);

    conveyrUpFollower.follow(conveyrLowLeader,true);

    conveyrLowLeader.burnFlash();
    conveyrUpFollower.burnFlash();

    sideSensor.setRangingMode(RangingMode.Short, 1);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("IN Side threshold", Constants.IntakeConstants.SideTreshholdIntake);
    SmartDashboard.putBoolean("IN Side triggered?", this.SideTriggered());
    SmartDashboard.putBoolean("IN Side Avg triggered?", this.SideAvgSensorTriggered());
    SmartDashboard.putNumber("IN Side distance", sideSensor.getRange());
    SmartDashboard.putNumber("IN Side Accumulator avg", this.averageSideRange());
    SmartDashboard.putNumber("IN Side sigma", sideSensor.getRangeSigma());

    SmartDashboard.putBoolean("IN either avg triggered?", this.EitherAvgSensorTriggered());
    SmartDashboard.putBoolean("IN either triggered?", this.EitherSensorTriggered());

    SmartDashboard.putBoolean("intake head is on", intake.get() > 0);
  }


  private double averageSideRange() {

    sideAccumulatorIndex++;
    sideAccumulatorIndex = sideAccumulatorIndex > 19 ? 0 : sideAccumulatorIndex;
    sideAccumulator[sideAccumulatorIndex] = sideSensor.getRange();

    double sideAvg = 0;
    for (int i = 0; i < 20; i++) {
      sideAvg += sideAccumulator[i];
    }
    return (sideAvg / 20);
  }


  public boolean SideAvgSensorTriggered() {
    return this.averageSideRange() <= Constants.IntakeConstants.SideTreshholdIntake;
  }

  public boolean SideTriggered() {
    return sideSensor.getRange() <= Constants.IntakeConstants.SideTreshholdIntake;
  }

  public boolean EitherSensorTriggered() {
    return this.SideTriggered();
  }

  public boolean EitherAvgSensorTriggered() {
    return this.SideAvgSensorTriggered();
  }

  public void setIntakeSpeed(double in_speed){
    intake.set(in_speed); 
  }

  public void setConveyorSpeed(double con_speed){
    conveyrLowLeader.set(con_speed);
  }
}
