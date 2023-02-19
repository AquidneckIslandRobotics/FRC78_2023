// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/**
 * implement the PID control loop for arm
 */
public class SetArmPID extends CommandBase {
  private Arm m_arm;

  /**
   * creates a command that takes in arm subsystem and sets shoulder and elbow to specific position
   * @param arm arm subsystem that this command requires
   */
  public SetArmPID(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    m_arm = arm;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderSpeed = m_arm.shoulderPIDcontroller.calculate(m_arm.getShoulderAbsolutePosition(), m_arm.shoulderTarget);
    if (shoulderSpeed < 0){
      shoulderSpeed = shoulderSpeed * 0.15;
    }
    m_arm.setShoulderSpeed(shoulderSpeed);
    
    double elbowSpeed = m_arm.elbowPIDcontroller.calculate(m_arm.getElbowAbsolutePosition(), m_arm.elbowTarget);
    if (elbowSpeed > 0){
      elbowSpeed = elbowSpeed * 0.5;
    }
    m_arm.setElbowSpeed(elbowSpeed * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setShoulderSpeed(0);
    m_arm.setElbowSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
