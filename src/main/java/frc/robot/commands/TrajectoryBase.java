// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.TrajectoryManager;
import frc.robot.subsystems.Drivetrain;

import java.sql.Driver;

public class TrajectoryBase extends CommandBase {
  /** Creates a new TrajectoryBase. */
    
  private Drivetrain m_drivetrain;
  private Trajectory m_trajectory;
  private String m_trajectoryName;

  private final Timer m_timer = new Timer();

  private final RamseteController m_ramsete = new RamseteController();

  private boolean isBackwards;
  private boolean resetGyro;
  private boolean m_isInitialized;

  public TrajectoryBase(Drivetrain drivetrain, String trajectory_name) {
    this(drivetrain, trajectory_name, false, true);
  }
  
  public TrajectoryBase(Drivetrain drivetrain, String trajectory_name, boolean isBackwards, boolean resetGyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    this.m_trajectoryName = trajectory_name;
    this.isBackwards = isBackwards;
    this.resetGyro = resetGyro;
  }

  private void getTrajectory() {
    if (m_trajectory == null && TrajectoryManager.getTrajectories() != null) {
      m_trajectory = TrajectoryManager.getTrajectories().get(m_trajectoryName);
    }
  }

  private void inializeTrajectory() {
    if (!m_isInitialized) {
      getTrajectory();
      if (m_trajectory != null) {
        m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
        m_isInitialized = true;
        
        m_timer.reset();
        m_timer.start();
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.printf("INFO: TrajectoryBase initialize: %s%n", m_trajectoryName);
    if (resetGyro) {
      m_drivetrain.resetGyro();
    } else {
      m_drivetrain.resetGyro180();
    }

    m_ramsete.setEnabled(true);
    m_drivetrain.setOdometryDirection(isBackwards);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsed = m_timer.get();

    inializeTrajectory();

    if (!m_isInitialized) { return; }

    Trajectory.State reference = m_trajectory.sample(elapsed);
      
    ChassisSpeeds speeds = m_ramsete.calculate(m_drivetrain.getPose(), reference);

    // var ramsete_speed = speeds.vxMetersPerSecond/Constants.MAX_SPEED_LOW_GEAR;
    var ramsete_speed = speeds.vxMetersPerSecond;
    var ramsete_rot = speeds.omegaRadiansPerSecond;

    var normalized_ramsete_speed = ramsete_speed / Drivetrain.MAX_SPEED;
    var normalized_ramsete_rot = -ramsete_rot / Drivetrain.MAX_ANGULAR_SPEED;

    var direction = isBackwards? -1.0 : 1.0;
    m_drivetrain.drive(normalized_ramsete_speed * direction, normalized_ramsete_rot);

    var t_pose = reference.poseMeters;
    var t_x = t_pose.getX();
    var t_y = t_pose.getY();
    var t_rotation = t_pose.getRotation().getDegrees();

    var a_pose = m_drivetrain.getPose();
    var a_x = a_pose.getX();
    var a_y = a_pose.getY();
    var a_rotation = a_pose.getRotation().getDegrees();
/*
    SmartDashboard.putNumber("Ramsete Speed - Normalized", normalized_ramsete_speed);
    SmartDashboard.putNumber("Ramsete Rot - Normalized", normalized_ramsete_rot);

    SmartDashboard.putNumber("Pose X - Trajectory", t_x);
    SmartDashboard.putNumber("Pose Y - Trajectory", t_y);
    SmartDashboard.putNumber("Pose R - Trajectory", t_rotation);

    SmartDashboard.putNumber("Pose X - Actual", a_x);
    SmartDashboard.putNumber("Pose Y - Actual", a_y);
    SmartDashboard.putNumber("Pose R - Actual", a_rotation);
*/
    System.out.printf("INFO: PoseX: %f, PoseY: %f, Rot: %f%n", t_x, t_y, t_rotation);
    m_drivetrain.fieldSim.setRobotPose(reference.poseMeters);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.printf("INFO: TrajectoryBase end: %s%n", m_trajectoryName);
    m_drivetrain.setOdometryDirection(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_trajectory != null) {
      return m_timer.get() > m_trajectory.getTotalTimeSeconds();
    }
    return false;
  }
}
