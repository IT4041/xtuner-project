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

  private double[] topAccumulator = new double[20];
  private double[] sideAccumulator = new double[20];
  private int topAccumulatorIndex = 0;
  private int sideAccumulatorIndex = 0;

  private final TimeOfFlight sideSensor = new TimeOfFlight(Constants.IntakeConstants.TimeOfFlightSideSensorID);
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

    sideSensor.setRangingMode(RangingMode.Short, 1);
    topSensor.setRangingMode(RangingMode.Short, 1);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("IN Top threshold", Constants.IntakeConstants.TopTreshholdIntake);
    SmartDashboard.putBoolean("IN Top triggered?", this.TopTriggered());
    SmartDashboard.putBoolean("IN Top Avg triggered?", this.TopAvgSensorTriggered());
    SmartDashboard.putNumber("IN Top Accumulator avg", this.averageTopRange());
    SmartDashboard.putNumber("IN Top distance", topSensor.getRange());
    SmartDashboard.putNumber("IN Top sigma", topSensor.getRangeSigma());

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

  private double averageTopRange() {

    topAccumulatorIndex++;
    topAccumulatorIndex = topAccumulatorIndex > 19 ? 0 : topAccumulatorIndex;
    topAccumulator[topAccumulatorIndex] = topSensor.getRange();

    double topAvg = 0;
    for (int i = 0; i < 20; i++) {
      topAvg += topAccumulator[i];
    }
    return (topAvg / 20);
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

  public boolean TopAvgSensorTriggered() {
    return this.averageTopRange() <= Constants.IntakeConstants.TopTreshholdIntake;
  }

  public boolean SideAvgSensorTriggered() {
    return this.averageSideRange() <= Constants.IntakeConstants.SideTreshholdIntake;
  }

  public boolean SideTriggered() {
    return sideSensor.getRange() <= Constants.IntakeConstants.SideTreshholdIntake;
  }

  public boolean TopTriggered() {
    return topSensor.getRange() <= Constants.IntakeConstants.TopTreshholdIntake;
  }

  public boolean EitherSensorTriggered() {
    return this.SideTriggered(); //|| this.TopTriggered();
  }

  public boolean EitherAvgSensorTriggered() {
    return this.SideAvgSensorTriggered() || this.TopAvgSensorTriggered();
  }

  public void on() {
    conveyrLow.set(Constants.IntakeConstants.ConveyrMotorSpeed);
    conveyrUp.set(-Constants.IntakeConstants.ConveyrMotorSpeed);
    intake.set(Constants.IntakeConstants.IntakeMotorSpeed);
  }

  public void off() {
    intake.stopMotor();
    conveyrLow.stopMotor();
    conveyrUp.stopMotor();
  }

  public void setIntakeSpeed(double in_speed){
    intake.set(in_speed); 
  }

  public void setConveyorSpeed(double con_speed){
    conveyrUp.set(con_speed);
    conveyrLow.set(con_speed);
  }
}
