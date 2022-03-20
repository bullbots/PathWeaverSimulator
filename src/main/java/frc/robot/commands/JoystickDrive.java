/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class JoystickDrive extends CommandBase {
  
  private Drivetrain m_drivetrain;
  private DoubleSupplier joyY;
  private DoubleSupplier joyX;
  private DoubleSupplier joyZ;

  public JoystickDrive(Drivetrain drivetrain, DoubleSupplier joyY, DoubleSupplier joyX) {
    // m_drivetrain = drivetrain;
    // this.joyY = joyY;
    // this.joyX = joyX;

    // addRequirements(m_drivetrain);
    this(drivetrain, joyY, joyX,  () -> 1.0);
  }

  public JoystickDrive(Drivetrain drivetrain, DoubleSupplier joyY, DoubleSupplier joyX, DoubleSupplier joyZ) {
    m_drivetrain = drivetrain;
    this.joyY = joyY;
    this.joyX = joyX;
    this.joyZ = joyZ;

    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double _joyY = -joyY.getAsDouble();
    double _joyX = -joyX.getAsDouble();
    m_drivetrain.drive(_joyY, _joyX);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
