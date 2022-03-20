// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase
{
    // 3 meters per second.
    public static final double MAX_SPEED = 3.0;
    // 1/2 rotation per second.
    public static final double MAX_ANGULAR_SPEED = Math.PI;
    
    private static final double TRACK_WIDTH = 0.381 * 2;
    private static final double WHEEL_RADIUS = 0.0508;
    private static final int ENCODER_RESOLUTION = -4096;
    
    private final PWMSparkMax leftLeader = new PWMSparkMax(1);
    private final PWMSparkMax leftFollower = new PWMSparkMax(2);
    private final PWMSparkMax rightLeader = new PWMSparkMax(3);
    private final PWMSparkMax rightFollower = new PWMSparkMax(4);
    
    private final MotorControllerGroup leftGroup =
            new MotorControllerGroup(leftLeader, leftFollower);
    private final MotorControllerGroup rightGroup =
            new MotorControllerGroup(rightLeader, rightFollower);
    
    private final Encoder leftEncoder = new Encoder(0, 1);
    private final Encoder rightEncoder = new Encoder(2, 3);
    
    private final PIDController leftPIDController = new PIDController(8.5, 0, 0);
    private final PIDController rightPIDController = new PIDController(8.5, 0, 0);
    
    private final AnalogGyro gyro = new AnalogGyro(0);
    
    private final DifferentialDriveKinematics kinematics =
            new DifferentialDriveKinematics(TRACK_WIDTH);
    private final DifferentialDriveOdometry odometry =
            new DifferentialDriveOdometry(gyro.getRotation2d());
    
    // Gains are for example purposes only - must be determined for your own
    // robot!
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);
    
    // Simulation classes help us simulate our robot
    private final AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);
    private final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
    private final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
    public final Field2d fieldSim = new Field2d();
    private final LinearSystem<N2, N2, N2> drivetrainSystem =
            LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    private final DifferentialDrivetrainSim drivetrainSimulator =
            new DifferentialDrivetrainSim(
                    drivetrainSystem, DCMotor.getCIM(2), 8, TRACK_WIDTH, WHEEL_RADIUS, null);

    private boolean flippedOdometry;
    
    /** Subsystem constructor. */
    public Drivetrain()
    {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
//        rightGroup.setInverted(true);
        
        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        leftEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);
        rightEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);
        
        leftEncoder.reset();
        rightEncoder.reset();
        
//        rightGroup.setInverted(true);
        SmartDashboard.putData("Field", fieldSim);
    }
    
    
    /** Sets speeds to the drivetrain motors. */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds)
    {
        var leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
        double leftOutput =
                leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
        double rightOutput =
                rightPIDController.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);

//        System.out.printf("INFO: leftMetersPerSecond: %f, rightMetersPerSecond: %f,%n", speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
//        System.out.printf("INFO: leftEncoder.getRate: %f, rightEncoder.getRate: %f,%n", leftEncoder.getRate(), rightEncoder.getRate());
//        System.out.printf("INFO: leftOutput: %f, rightOutput: %f,%n", leftOutput , rightOutput);
        leftGroup.setVoltage(leftOutput + leftFeedforward);
        rightGroup.setVoltage(rightOutput + rightFeedforward);
//        leftGroup.setVoltage(1.0);
//        rightGroup.setVoltage(1.0);
    }
    
    
    /**
     * Controls the robot using arcade drive.
     *
     * @param xSpeed   the speed for the x axis
     * @param rotation the rotation
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rotation)
    {
//        System.out.printf("INFO: xSpeed: %f, rotation: %f%n", xSpeed, rotation);
        setSpeeds(kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rotation)));
    }

    public void stop() {
        setSpeeds(kinematics.toWheelSpeeds(
                new ChassisSpeeds(0.0, 0.0, 0.0)));
    }
    
    /** Update robot odometry. */
    public void updateOdometry()
    {
        odometry.update(
                gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }
    
    
    /** Resets robot odometry. */
    public void resetOdometry(Pose2d pose)
    {
        leftEncoder.reset();
        rightEncoder.reset();
        drivetrainSimulator.setPose(pose);
        odometry.resetPosition(pose, gyro.getRotation2d());
    }
    
    
    /** Check the current robot pose. */
    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }
    
    
    /** Update our simulation. This should be run every robot loop in simulation. */
    public void simulationPeriodic()
    {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.

//        System.out.printf("INFO: leftLeader.get(): %f, rightLeader.get(): %f%n", leftLeader.get(), rightLeader.get());
        drivetrainSimulator.setInputs(
                leftLeader.get() * RobotController.getInputVoltage(),
                rightLeader.get() * RobotController.getInputVoltage());
        drivetrainSimulator.update(0.02);
        
        leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
        leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
        rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-drivetrainSimulator.getHeading().getDegrees());
    }

    public void setOdometryDirection(boolean invert) {
        flippedOdometry = invert;
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetGyro180() {
//        gyro.reset180();
        gyro.reset();
    }
    
    /** Update odometry - this should be run every robot loop. */
    public void periodic()
    {
        updateOdometry();
        fieldSim.setRobotPose(odometry.getPoseMeters());
        /*
        var pose = odometry.getPoseMeters();
        var poseTranslation = pose.getTranslation();
        var poseX = poseTranslation.getX();
        var poseY = poseTranslation.getY();
        var poseRotDegrees = pose.getRotation().getDegrees();
        System.out.printf("INFO: poseX: %f, poseY: %f, Rot: %f%n", poseX, poseY, poseRotDegrees);
        */
    }
}
