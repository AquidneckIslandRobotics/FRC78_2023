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
  private Arm arm;
  /**
   * creates a command that takes in arm subsystem and sets shoulder and elbow to specific position
   * @param arm arm subsystem that this command requires
   */
  public SetArmPID(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderSpeed = arm.shoulderPIDcontroller.calculate(arm.getShoulderAbsolutePosition(), arm.shoulderTarget);
    if (shoulderSpeed < 0){
      shoulderSpeed = shoulderSpeed * 0.15;
    }
    arm.setShoulderSpeed(arm.isLimitShoulder() && shoulderSpeed < 0 ? 0 : shoulderSpeed * -1);
    
    double elbowSpeed = arm.elbowPIDcontroller.calculate(arm.getElbowAbsolutePosition(), arm.elbowTarget);
    if (elbowSpeed > 0){
      elbowSpeed = elbowSpeed * 0.5;
    }
    arm.setElbowSpeed(arm.isLimitShoulder() && shoulderSpeed < 0 ? 0 : elbowSpeed * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setShoulderSpeed(0);
    arm.setElbowSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
