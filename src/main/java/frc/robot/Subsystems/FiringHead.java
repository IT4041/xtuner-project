// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FiringHead extends SubsystemBase {

  private CANSparkMax fireMotor;
  private CANSparkMax followMotor;

  private final TimeOfFlight centerSensor = new TimeOfFlight(Constants.FiringHeadConstants.TimeOfFlightASensorID);
  private final TimeOfFlight sideSensor = new TimeOfFlight(Constants.FiringHeadConstants.TimeOfFlightBSensorID);

  private CANSparkMax transportMotor;
  private RelativeEncoder m_Encoder;

  private double[] centerAccumulator = new double[20];
  private double[] sideAccumulator = new double[20];
  private int centerAccumulatorIndex = 0;
  private int sideAccumulatorIndex = 0;

  /** Creates a new FiringHead. */
  public FiringHead() {
    fireMotor = new CANSparkMax(Constants.FiringHeadConstants.UpperSparkmaxDeviceID, MotorType.kBrushless);
    followMotor = new CANSparkMax(Constants.FiringHeadConstants.LowerSparkmaxDeviceID, MotorType.kBrushless);
    transportMotor = new CANSparkMax(Constants.FiringHeadConstants.UpperTransportSparkmaxDeviceID,
        MotorType.kBrushless);

    fireMotor.restoreFactoryDefaults();
    followMotor.restoreFactoryDefaults();
    transportMotor.restoreFactoryDefaults();

    m_Encoder = fireMotor.getEncoder();

    fireMotor.setIdleMode(IdleMode.kCoast);
    fireMotor.setSmartCurrentLimit(80);
    fireMotor.setClosedLoopRampRate(1);

    followMotor.setIdleMode(IdleMode.kCoast);
    followMotor.setSmartCurrentLimit(80);
    followMotor.setClosedLoopRampRate(1);

    transportMotor.setIdleMode(IdleMode.kBrake);
    transportMotor.setSmartCurrentLimit(80);
    transportMotor.setClosedLoopRampRate(1);

    followMotor.follow(fireMotor, true);

    centerSensor.setRangingMode(RangingMode.Short, 50);
    sideSensor.setRangingMode(RangingMode.Short, 50);

    fireMotor.burnFlash();
    followMotor.burnFlash();
    transportMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    this.averageCenterRange();
    this.averageSideRange();

    SmartDashboard.putNumber("firing head velocity", m_Encoder.getVelocity());

    SmartDashboard.putNumber("SH Center threshold", Constants.FiringHeadConstants.CenterSensorThreshold);
    SmartDashboard.putBoolean("SH Center triggered?", this.CenterSensorTriggered());
    SmartDashboard.putBoolean("SH Center Avg triggered?", this.CenterAvgSensorTriggered());
    SmartDashboard.putNumber("SH Center Accumulator avg", this.averageCenterRange());
    SmartDashboard.putNumber("SH Center distance", centerSensor.getRange());
    SmartDashboard.putNumber("SH Center sigma", centerSensor.getRangeSigma());

    SmartDashboard.putNumber("SH Side threshold", Constants.FiringHeadConstants.SideSensorThreshold);
    SmartDashboard.putBoolean("SH Side triggered?", this.SideSensorTriggered());
    SmartDashboard.putBoolean("SH Side Avg triggered?", this.SideAvgSensorTriggered());
    SmartDashboard.putNumber("SH Side distance", sideSensor.getRange());
    SmartDashboard.putNumber("SH Side Accumulator avg", this.averageSideRange());
    SmartDashboard.putNumber("SH Side sigma", sideSensor.getRangeSigma());

    SmartDashboard.putBoolean("conveyr on", transportMotor.get() > 0);
    SmartDashboard.putBoolean("shooter head is on", fireMotor.get() > 0);

  }

  private double averageCenterRange() {

    centerAccumulatorIndex++;
    centerAccumulatorIndex = centerAccumulatorIndex > 19 ? 0 : centerAccumulatorIndex;
    centerAccumulator[centerAccumulatorIndex] = centerSensor.getRange();

    double centerAvg = 0;
    for (int i = 0; i < 20; i++) {
      centerAvg += centerAccumulator[i];
    }
    return (centerAvg / 20);
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

  public void Feed() {
    transportMotor.set(Constants.FiringHeadConstants.TransportMotorSpeed);
    fireMotor.set(Constants.FiringHeadConstants.FiringSpeed);
  }

  public void Source() {
    transportMotor.set(Constants.FiringHeadConstants.SourceTransportMotorSpeed);
    fireMotor.set(Constants.FiringHeadConstants.SourceSpeed);
  }

  public void StopTransport() {
    transportMotor.stopMotor();
  }

  public boolean CenterAvgSensorTriggered() {
    return this.averageCenterRange() <= Constants.FiringHeadConstants.CenterSensorThreshold;
  }

  public boolean CenterSensorTriggered() {
    return centerSensor.getRange() <= Constants.FiringHeadConstants.CenterSensorThreshold;
  }

  public boolean SideAvgSensorTriggered() {
    return this.averageSideRange() <= Constants.FiringHeadConstants.SideSensorThreshold;
  }

  public boolean SideSensorTriggered() {
    return sideSensor.getRange() <= Constants.FiringHeadConstants.SideSensorThreshold;
  }

  public boolean EitherSensorTriggered() {
    return this.SideSensorTriggered() || this.CenterSensorTriggered();
  }

  public boolean EitherAvgSensorTriggered() {
    return this.SideAvgSensorTriggered() || this.CenterAvgSensorTriggered();
  }

  public void MasterStop() {
    transportMotor.stopMotor();
    fireMotor.stopMotor();
  }

  public void shooterSetSpeed(double speed) {
    fireMotor.set(speed);
  }

  public void setTransportMotorSpeed(double speed) {
    transportMotor.set(speed);
  }
}
