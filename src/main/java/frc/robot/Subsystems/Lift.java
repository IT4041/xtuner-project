// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Lift extends SubsystemBase {

    private CANSparkMax lift;
    private RelativeEncoder m_Encoder;
    private DigitalInput limit_switch;

    public Lift() {
        lift = new CANSparkMax(Constants.LiftConstants.LiftSparkmaxDeviceID, MotorType.kBrushless);
        lift.restoreFactoryDefaults();
        lift.setIdleMode(IdleMode.kBrake);
        lift.setSmartCurrentLimit(80);
        lift.setInverted(true);
        limit_switch = new DigitalInput(Constants.LiftConstants.limitSwitchPort);

        m_Encoder = lift.getEncoder();
        //ResetEncoder();

        lift.setSoftLimit(SoftLimitDirection.kForward, Constants.LiftConstants.Extended);
        lift.setSoftLimit(SoftLimitDirection.kReverse, Constants.LiftConstants.Home);
        lift.enableSoftLimit(SoftLimitDirection.kForward, true);
        lift.enableSoftLimit(SoftLimitDirection.kReverse, true);
        lift.burnFlash();
        SmartDashboard.putNumber("Lift", m_Encoder.getPosition());
    }

    public void periodic() {

        SmartDashboard.putNumber("Lift", m_Encoder.getPosition());
        SmartDashboard.putBoolean("get limit switch state", getLimitSwitchState());

        if (getLimitSwitchState()) {
            if (lift.get() < 0) {
                lift.stopMotor();
            }
            //ResetEncoder();
        }
    }

    public void up() {
        lift.set(Constants.LiftConstants.up_speed);
    }

    public void down() {
        double speed = Constants.LiftConstants.down_speed;
        if (m_Encoder.getPosition() < 100) {
            speed = speed / 2;
        }
        lift.set(speed);

    }

    public void stop() {
        lift.stopMotor();
    }

    public boolean getLimitSwitchState() {
        return !limit_switch.get();
    }

    private void ResetEncoder() {
        m_Encoder.setPosition(Constants.LiftConstants.Home);
    }
}
