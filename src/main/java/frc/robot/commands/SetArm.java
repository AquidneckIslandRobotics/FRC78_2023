// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
  private double elbowTarget;
  private double shoulderTarget;
  private Arm arm;
  
  
  /** Creates a new RunArmToTarget. */
  public SetArm(Arm arm, double elbowTarget, double shoulderTarget) {
    this.arm = arm;
    this.elbowTarget = elbowTarget;
    this.shoulderTarget = shoulderTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.elbowPIDcontroller.setSetpoint(elbowTarget);
    arm.shoulderPIDcontroller.setSetpoint(shoulderTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //  arm.elbowGoToPosition(elbowTarget);
    //arm.shoulderGoToPosition(shoulderTarget);
    arm.elbowTarget = elbowTarget;
    arm.shoulderTarget = shoulderTarget;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.shoulderPIDcontroller.reset();
    arm.elbowPIDcontroller.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.print("Shoulder at pos? :" + arm.shoulderPIDcontroller.atSetpoint() + " elbow: " + arm.elbowPIDcontroller.atSetpoint());
    // return arm.shoulderPIDcontroller.atSetpoint() && arm.elbowPIDcontroller.atSetpoint();
    return Math.abs(arm.shoulderPIDcontroller.getPositionError()) < 2 && Math.abs(arm.elbowPIDcontroller.getPositionError()) < 2;
  }
}