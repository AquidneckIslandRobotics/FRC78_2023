// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

  @Override
  public void initialize() { 
    arm.elbowTarget = elbowTarget;
    arm.shoulderTarget = shoulderTarget;
    arm.lastTargetChangeTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.print("SetArm command ended!");
    arm.shoulderPIDcontroller.reset();
    arm.elbowPIDcontroller.reset();
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(arm.getElbowAbsolutePosition() - this.elbowTarget) < 2) && (Math.abs(arm.getShoulderAbsolutePosition() - this.shoulderTarget) < 2)
    || arm.elbowTarget != this.elbowTarget || arm.shoulderTarget != this.shoulderTarget;
  }
}