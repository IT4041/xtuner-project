// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

public class Constants {

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final class PivotConstants {

        public static int SparkmaxDeviceID = 8;

        public static final class PivotPIDConstants {
            public static final double kP = 0.02;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double period = 0.001;
        }

        public static final class PivotPostions {
            
            public static final double ZeroOffset = 124.4;
            
            public static final double ShootingPointMidRange = 41.5; //auto shooting pos 2 39.5
            public static final double DumpPoint = 36;
            public static final double ShootingPointShortRange = 30; //auto shooting pos 1
            public static final double StartingPoint = 21; //up close shooting
            public static final double[] PivotPoses = {StartingPoint, ShootingPointShortRange, DumpPoint, ShootingPointMidRange};
        }
    }

    public static final class FiringHeadConstants {

        public static int UpperSparkmaxDeviceID = 9;
        public static int LowerSparkmaxDeviceID = 55;
        public static int UpperTransportSparkmaxDeviceID = 33;

        public static int TimeOfFlightASensorID = 89;
        public static int TimeOfFlightBSensorID = 90;

        public static int CenterSensorThreshold = 80;
        public static int SideSensorThreshold = 40;

        public static double NearFiringSpeed = 0.41;
        public static double DumpSpeed = 0.3;
        public static double FarFiringSpeed = 0.45;

        public static double TransportMotorSpeed = 0.5;

        public static double SourceSpeed = -0.3;
        public static double SourceTransportMotorSpeed = -0.4;

        public static final class FiringHeadPIDConstants {
            
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kIz = 0;
            public static final double kFF = 0;
            public static final double kMaxOutput = 0;
            public static final double kMinOutput = 0;
        }
    }

    public static final class IntakeConstants {

        public static int LowerIntakeSparkmaxDeviceID = 11;
        public static int UpperConvyerSparkmaxDeviceID = 1;
        public static int LowerConvyerSparkmaxDeviceID = 2;
        public static int UpperIntakeSparkmaxDeviceID = 33;

        public static int TimeOfFlightSideSensorID = 88;
        public static int TimeOfFlightTopSensorID = 87;

        public static double SideTreshholdIntake = 75;
        public static double TopTreshholdIntake = 300;

        public static double IntakeMotorSpeed = 0.7; // Number Reduced For Testing  Original 1
        public static double ConveyrMotorSpeed = 0.4; // 0.8

    }

    public static final class LiftConstants{
        public static final int LiftSparkmaxDeviceID = 32;
        public static final int Home = 0;
        public static final int Extended = 582;
        public static final double up_speed = 1.0;
        public static final double down_speed = -1.0;
        public static final int limitSwitchPort = 5;
        
    }

    public static final class LEDConstants{
        public static final int LED_PWM_PORT = 6;
        public static final int LED_LENGTH = 50;
    
        public static final Color purple = new Color(255, 1, 255);
        public static final Color yellow = new Color(255, 128, 1);
        public static final Color orange = new Color(255, 55, 5);
        public static final Color pink = new Color(255, 50, 30);
        public static final Color green = new Color(1, 255, 1);
        public static final Color red = new Color(255, 1, 1);

    }
}
