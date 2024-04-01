package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final Field2d m_field = new Field2d();

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.buildAutoBuilder();
        SmartDashboard.putData("Field", m_field);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.buildAutoBuilder();
        SmartDashboard.putData("Field", m_field);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {

        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.buildAutoBuilder();
        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {

        // if(!DriverStation.isAutonomousEnabled()){
        //     LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        //     if (limelightMeasurement.tagCount >= 2) {
        //         super.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2.5)));
        //         super.addVisionMeasurement(
        //                 limelightMeasurement.pose,
        //                 limelightMeasurement.timestampSeconds);
        //     }
        // }

        Pose2d currPose2d = super.getState().Pose;
        m_field.setRobotPose(currPose2d);

        SmartDashboard.putNumber("Pose Y", currPose2d.getY());
        SmartDashboard.putNumber("Pose X", currPose2d.getX());
        SmartDashboard.putNumber("Pose Rot", currPose2d.getRotation().getDegrees());
        SmartDashboard.putNumber("TA", LimelightHelpers.getTA("limelight"));
        SmartDashboard.putNumber("TX", LimelightHelpers.getTX("limelight"));
        SmartDashboard.putNumber("TY", LimelightHelpers.getTY("limelight"));
        SmartDashboard.putBoolean("Pose Rot", LimelightHelpers.getTV("limelight"));

        SmartDashboard.putBoolean("is Red", this.isRed());
        SmartDashboard.putNumber("poseY", this.getState().Pose.getY());
    }

    private void buildAutoBuilder() {

        // double driveBaseRadius = 0;
        // for (var moduleLocation : m_moduleLocations) {
        // driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        // }

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(new PIDConstants(4.0, 0, 0), // translation constant
                        new PIDConstants(5.0, 0, 0), // rotation constant
                        TunerConstants.kSpeedAt12VoltsMps,
                        this.getDriveBaseRadius(),
                        new ReplanningConfig()),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void commandForMovement(double velocityX, double velocityY, double rotationalRate,
            SwerveRequest.FieldCentric drive) {
        this.applyRequest(() -> drive.withVelocityX(velocityX) // Drive forward with negative Y (forward)
                .withVelocityY(velocityY) // Drive left with negative X (left)
                .withRotationalRate(rotationalRate)); // Drive counterclockwise with negative X (left)
    }

    public void goForward(SwerveRequest.FieldCentric drive) {
        // this.applyRequest(() -> drive.withVelocityX(-0.25) // Drive forward with
        // // negative Y (forward)
        // .withVelocityY(0) // Drive left with negative X (left)
        // .withRotationalRate(0)); // Drive counterclockwise with negative X (left)
        this.commandForMovement(-0.2, 0, 0, drive);
    }

    public void stop(SwerveRequest.FieldCentric drive) {
        this.commandForMovement(0, 0, 0, drive);
    }

    private double getDriveBaseRadius() {

        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        return driveBaseRadius;
    }

    private boolean isRed(){
        var alliance = DriverStation.getAlliance();
        boolean isRed = false;

        if (alliance.isPresent()) {
            isRed =  (alliance.get() == DriverStation.Alliance.Red);
        }
        return isRed;
    }

    private PathPlannerPath getPathAmp(){

        boolean isRed = this.isRed();
        PathPlannerPath path = null;

        if(!isRed){
            path = PathPlannerPath.fromPathFile("blue amp side");
        }
        else{
            path = PathPlannerPath.fromPathFile("red amp side");
        }

        return path;
    }

    private PathPlannerPath getPathSource(){

        boolean isRed = this.isRed();
        PathPlannerPath path = null;

        if(!isRed){
            path = PathPlannerPath.fromPathFile("blue source side");
        }
        else{
            path = PathPlannerPath.fromPathFile("red source side");
        }

        return path;
    }

    public Command autoAlignSource() {

        SmartDashboard.putBoolean("is Red", this.isRed());

        // Load the path we want to pathfind to and follow
        PathPlannerPath path = this.getPathSource();

        // Create the constraints to use while pathfinding. The constraints defined in
        // the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
                2.5, 3.5,
                Units.degreesToRadians(270), Units.degreesToRadians(360));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                0.01 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        return pathfindingCommand;
    }

        public Command autoAlignAmp() {

        // Load the path we want to pathfind to and follow
        PathPlannerPath path = this.getPathAmp();

        // Create the constraints to use while pathfinding. The constraints defined in
        // the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
                2.5, 3.5,
                Units.degreesToRadians(270), Units.degreesToRadians(360));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                0.01 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        return pathfindingCommand;
    }
}
