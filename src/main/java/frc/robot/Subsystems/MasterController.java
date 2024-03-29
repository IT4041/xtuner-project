// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MasterController extends SubsystemBase {

  private final Intake m_intake;
  private final Pivot m_pivot;
  private final FiringHead m_firingHead;
  private final LED m_led;
  private boolean override = false;

  public MasterController(Pivot in_pivot, Intake in_intake, FiringHead in_firingHead, LED in_led) {
    m_intake = in_intake;
    m_pivot = in_pivot;
    m_firingHead = in_firingHead;
    m_led = in_led;
  }

  @Override
  public void periodic() {

    // change leds if we have a note
    if (this.anySensorTriggered()) {
      m_led.capturedNote();
    } else {
      m_led.noNote();
    }

    
    //if we have a note in firing head move to short shooting position
    // and turn on shooting wheels
    if(m_firingHead.EitherSensorTriggered() && !DriverStation.isAutonomousEnabled()){
      if(m_pivot.InStartingPosition()){
        m_pivot.GoToShootingShortRange();
      }

      // wait for shooter wheels to stop spinning before switching direction
      // to avoid wear on motors and gearboxes
      if(m_firingHead.getShooterVelocity() > -0.05){
        m_firingHead.shooterSetSpeed(this.getFiringSpeed());
      }
    }

    //shuffle note that was stopped in intake up to shooterhead as soon as pivot is in starting position
    // if(m_intake.EitherSensorTriggered() && m_pivot.InStartingPosition()){
    //   SequentialCommandGroup  shuffle = new RunCommand(() -> this.runConveyors(), this)
    //     .until(() -> (m_firingHead.CenterSensorTriggered() && m_pivot.InStartingPosition())
    //         || m_operatorController.leftTrigger().getAsBoolean()
    //         || m_operatorController.start().getAsBoolean()
    //         || m_driverController.start().getAsBoolean())
    //     .andThen(new InstantCommand(() -> this.stopConveyors(), this));
    //     shuffle.execute();
    // }

    SmartDashboard.putBoolean("MC any avg triggered?", this.anySensorTriggered());
    SmartDashboard.putBoolean("MC any triggered?", this.anyAvgSensorTriggered());
  }

  public void runConveyors() {
    m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.TransportMotorSpeed);
    m_intake.setConveyorSpeed(Constants.IntakeConstants.ConveyrMotorSpeed);
    m_intake.setIntakeSpeed(Constants.IntakeConstants.IntakeMotorSpeed);
  }

  public void stopConveyors() {
    m_firingHead.setTransportMotorSpeed(0);
    m_intake.setConveyorSpeed(0);
    m_intake.setIntakeSpeed(0);
  }

  public double getFiringSpeed() {

    double retSpeed;
    if(override)
    {
       retSpeed = Constants.FiringHeadConstants.MaxSpeed;
    }
    else{
      switch (m_pivot.ReturnPositionIndex()) {
        case 3:
          retSpeed = Constants.FiringHeadConstants.FarFiringSpeed;
          break;
        case 2:
          retSpeed = Constants.FiringHeadConstants.DumpSpeed;
          break;
        case 1:
          retSpeed = Constants.FiringHeadConstants.NearFiringSpeed;
          break;
        case 0:
          retSpeed = Constants.FiringHeadConstants.NearFiringSpeed;
          break;
        default:retSpeed = Constants.FiringHeadConstants.NearFiringSpeed;
          break;
      }
    }
    return retSpeed;
  }

  public void OverrideOn(){
     override = true;
  }

    public void OverrideOff(){
     override = false;
  }

  private boolean anySensorTriggered() {
    return m_firingHead.EitherSensorTriggered() || m_intake.EitherSensorTriggered();
  }

    private boolean anyAvgSensorTriggered() {
    return m_firingHead.EitherAvgSensorTriggered() || m_intake.EitherAvgSensorTriggered();
  }

    public Object autoAlign() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'autoAlign'");
    }
}
