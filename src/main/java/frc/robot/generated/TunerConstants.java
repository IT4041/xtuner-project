package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.CommandSwerveDrivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
                        .withKP(100).withKI(0).withKD(0.2)
                        .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
                        .withKP(3).withKI(0).withKD(0)//3
                        .withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 300.0;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 5.33;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;;

        // MK4i drive gear ratios
        // L1 L2(using) L3
        // stage Driving Driven Driving Driven Driving Driven
        // 1 14 50 14 50 14 50
        // 2 25 19 27 17 28 16
        // 3 15 45 15 45 15 45
        // 8.14:1 6.75:1 6.12:1
        private static final double Stage1Ratio = 50.0 / 14.0;
        private static final double Stage2Ratio = 16.0 / 28.0;
        private static final double Stage3Ratio = 45.0 / 15.0;
        public static final double driveGearRatio = Stage1Ratio * Stage2Ratio * Stage3Ratio; // 6.12:1

        // The steering gear ratio of the MK4i is 150/7:1
        public static final double angleGearRatio = 21.428571428571427;;

        private static final double kDriveGearRatio = driveGearRatio;// 5.902777777777778;
        private static final double kSteerGearRatio = angleGearRatio;// 21.428571428571427;

        private static final double kWheelRadiusInches = 1.95;

        private static final boolean kSteerMotorReversed = true;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "CANivore1";
        private static final int kPigeonId = 13;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                        .withPigeon2Id(kPigeonId)
                        .withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                        .withDriveMotorGearRatio(kDriveGearRatio)
                        .withSteerMotorGearRatio(kSteerGearRatio)
                        .withWheelRadius(kWheelRadiusInches)
                        .withSlipCurrent(kSlipCurrentA)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                        .withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withSteerFrictionVoltage(kSteerFrictionVoltage)
                        .withDriveFrictionVoltage(kDriveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.RemoteCANcoder) //licened value: FusedCANcoder
                        .withCouplingGearRatio(kCoupleRatio)
                        .withSteerMotorInverted(kSteerMotorReversed);

        // Front Left
        private static final int kFrontLeftDriveMotorId = 24;
        private static final int kFrontLeftSteerMotorId = 22;
        private static final int kFrontLeftEncoderId = 53;
        private static final double kFrontLeftEncoderOffset = 0.32421875;

        private static final double kFrontLeftXPosInches = 11.125;
        private static final double kFrontLeftYPosInches = 13.375;

        // Front Right
        private static final int kFrontRightDriveMotorId = 23;
        private static final int kFrontRightSteerMotorId = 41;
        private static final int kFrontRightEncoderId = 52;
        private static final double kFrontRightEncoderOffset = 0.41064453125;

        private static final double kFrontRightXPosInches = 11.125;
        private static final double kFrontRightYPosInches = -13.375;

        // Back Left
        private static final int kBackLeftDriveMotorId = 43;
        private static final int kBackLeftSteerMotorId = 44;
        private static final int kBackLeftEncoderId = 54;
        private static final double kBackLeftEncoderOffset = -0.44140625;

        private static final double kBackLeftXPosInches = -11.125;
        private static final double kBackLeftYPosInches = 13.375;

        // Back Right
        private static final int kBackRightDriveMotorId = 45;
        private static final int kBackRightSteerMotorId = 40;
        private static final int kBackRightEncoderId = 50;
        private static final double kBackRightEncoderOffset = 0.1064453125;

        private static final double kBackRightXPosInches = -11.125;
        private static final double kBackRightYPosInches = -13.375;

        private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                        kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                        Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
                        kInvertLeftSide);
        private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                        kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                        kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
                        Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                        kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                        Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
                        kInvertLeftSide);
        private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                        kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                        Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
                        kInvertRightSide);

        public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants,250,
                        VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1.0)),//odometery stdev
                        VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1.0)),// vision stdev
                        FrontLeft,FrontRight, BackLeft, BackRight);
}
