// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmAuto extends CommandBase {
  private double elbowTarget;
  private double shoulderTarget;
  private Arm arm;
  
  
  /** Creates a new RunArmToTarget. */
  public SetArmAuto(Arm arm, double elbowTarget, double shoulderTarget) {
    this.arm = arm;
    this.elbowTarget = elbowTarget;
    this.shoulderTarget = shoulderTarget;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting");
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
    System.out.println("Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("shoulder" + arm.shoulderPIDcontroller.getPositionError());
    System.out.println("elbow " + arm.elbowPIDcontroller.getPositionError());
    //return arm.shoulderPIDcontroller.atSetpoint() && arm.elbowPIDcontroller.atSetpoint();
    return arm.elbowPIDcontroller.getPositionError() < 2 && arm.shoulderPIDcontroller.getPositionError() < 2;

  
  }
}