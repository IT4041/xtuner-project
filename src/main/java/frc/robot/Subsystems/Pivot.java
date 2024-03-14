// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

  private CANSparkMax mainMotor;
  private PIDController m_wPidController;
  private SparkAbsoluteEncoder m_Encoder;
  private double kP, kI, kD, period;
  private double wpi_pid_output;
  private double current_position,target_position;
  private int position_index = 0;

  public Pivot() {
    mainMotor = new CANSparkMax(Constants.PivotConstants.SparkmaxDeviceID, MotorType.kBrushless);
    mainMotor.restoreFactoryDefaults();
    mainMotor.setControlFramePeriodMs(0);

    m_Encoder = mainMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_Encoder.setPositionConversionFactor(180);
    m_Encoder.setVelocityConversionFactor(1);
    m_Encoder.setInverted(false);
    m_Encoder.setZeroOffset(Constants.PivotConstants.PivotPostions.ZeroOffset);

    kP = Constants.PivotConstants.PivotPIDConstants.kP;
    kI = Constants.PivotConstants.PivotPIDConstants.kI;
    kD = Constants.PivotConstants.PivotPIDConstants.kD;
    period = Constants.PivotConstants.PivotPIDConstants.period;

    m_wPidController = new PIDController(kP, kI, kD, period);

    mainMotor.setIdleMode(IdleMode.kBrake);
    mainMotor.setSmartCurrentLimit(60);
    mainMotor.setClosedLoopRampRate(2);
    mainMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.PivotConstants.PivotPostions.DumpPoint);
    mainMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.PivotConstants.PivotPostions.StartingPoint);
    mainMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    mainMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    mainMotor.enableVoltageCompensation(12);

    mainMotor.burnFlash();

    current_position = Constants.PivotConstants.PivotPostions.PivotPoses[readInitalPosition()];
    target_position = current_position;
  }

  public void periodic() {

    wpi_pid_output = m_wPidController.calculate(m_Encoder.getPosition(), target_position);
    mainMotor.set(wpi_pid_output);

    SmartDashboard.putNumber("Pivot encoder position", m_Encoder.getPosition());
    SmartDashboard.putNumber("Pivot encoder offset", m_Encoder.getZeroOffset());
    SmartDashboard.putNumber("current_position", current_position);
    SmartDashboard.putNumber("output", mainMotor.getAppliedOutput());
    SmartDashboard.putNumber("wpi_pid_output", wpi_pid_output);

    
    SmartDashboard.putBoolean("Shoot High", position_index == 3);
    SmartDashboard.putBoolean("Dump", position_index == 2);
    SmartDashboard.putBoolean("Shoot Low", position_index == 1);
    SmartDashboard.putBoolean("Starting", position_index == 0);
    SmartDashboard.putBoolean("Pivot inStartingPosition", this.InStartingPosition());
  }

  private int readInitalPosition(){
    double initialPosition = m_Encoder.getPosition();
    int pos = 0;//default to starting/intake position
    
    if(this.withinRange(2, initialPosition, Constants.PivotConstants.PivotPostions.ShootingPointShortRange)){
      pos = 1;
    }
    else if(this.withinRange(2, initialPosition, Constants.PivotConstants.PivotPostions.DumpPoint)){
      pos = 2;
    }
    else if(this.withinRange(2, initialPosition, Constants.PivotConstants.PivotPostions.ShootingPointMidRange)){
      pos = 3;
    }
    return pos;
  }

  private void setPosition(double position) {
    target_position = position;
  }

  public void GoToStarting() {
    current_position = Constants.PivotConstants.PivotPostions.StartingPoint;
    this.setPosition(current_position);
    position_index = 0;
  }

  public void GoToShootingShortRange() {
    current_position = (Constants.PivotConstants.PivotPostions.ShootingPointShortRange + 1.5);
    this.setPosition(current_position);
    position_index = 1;
  }

  public void GoToDump() {
    current_position = Constants.PivotConstants.PivotPostions.DumpPoint;
    this.setPosition(current_position);
    position_index = 2;
  }

  public void GoToShootingMidRange() {
    current_position = Constants.PivotConstants.PivotPostions.ShootingPointMidRange;
    this.setPosition(current_position);
    position_index = 3;
  }

  public void GoToLeftNoteShootingPosition(){
    current_position = (Constants.PivotConstants.PivotPostions.DumpPoint + 1.5);
    this.setPosition(current_position);
  }

  public void GoToCenterNoteShootingPosition(){
    current_position = (Constants.PivotConstants.PivotPostions.ShootingPointShortRange + 1.5);
    this.setPosition(current_position);
  }

  public void GoToRightNoteShootingPosition(){
    current_position = (Constants.PivotConstants.PivotPostions.DumpPoint + 1.5);
    this.setPosition(current_position);
  }

  public boolean InStartingPosition() {
    return withinRange(2.25, m_Encoder.getPosition(), Constants.PivotConstants.PivotPostions.StartingPoint);
  }

  private boolean withinRange(double range, double value, double compareAgainst){
   return compareAgainst+range >= value && compareAgainst-range < value;
  }

  public int ReturnPositionIndex(){
    return this.position_index;
  }

  public void up() {
    if (position_index < 3) {
      position_index++;
    }
    this.current_position = Constants.PivotConstants.PivotPostions.PivotPoses[position_index];
    this.setPosition(current_position);
  }

  public void down() {
    if (position_index > 0) {
      position_index--;
    }
    this.current_position = Constants.PivotConstants.PivotPostions.PivotPoses[position_index];
    this.setPosition(current_position);
  }
}
