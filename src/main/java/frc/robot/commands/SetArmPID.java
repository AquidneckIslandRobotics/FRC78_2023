// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/**
 * implement the PID control loop for arm
 */
public class SetArmPID extends CommandBase {
  private Arm arm;

  private TrapezoidProfile.State elbow_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State elbow_currentPos = new TrapezoidProfile.State();
  private TrapezoidProfile.State shoulder_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State shoulder_currentPos = new TrapezoidProfile.State();
  private double calcElbowGoal;
  private double calcShoulderGoal;

  public SetArmPID(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elbow_goal = new TrapezoidProfile.State(arm.elbowTarget, 0);
    shoulder_goal = new TrapezoidProfile.State(arm.shoulderTarget, 0);
    elbow_currentPos = new TrapezoidProfile.State(arm.getElbowAbsolutePosition(), 0);
    shoulder_currentPos = new TrapezoidProfile.State(arm.getShoulderAbsolutePosition(), 0);
    calcElbowGoal = arm.elbowProfile.calculate(Timer.getFPGATimestamp() - arm.lastTargetChangeTimestamp).position;
    calcShoulderGoal = arm.shoulderProfile.calculate(Timer.getFPGATimestamp() - arm.lastTargetChangeTimestamp).position;
    double shoulderSpeed = arm.shoulderPIDcontroller.calculate(arm.getShoulderAbsolutePosition(), calcShoulderGoal); 
    double elbowSpeed = arm.elbowPIDcontroller.calculate(arm.getElbowAbsolutePosition(), calcElbowGoal);

    SmartDashboard.putNumber("TrapElbowGoal", arm.elbowProfile.calculate(Timer.getFPGATimestamp() - arm.lastTargetChangeTimestamp).velocity);
    SmartDashboard.putNumber("TrapShoulderGoal", arm.shoulderProfile.calculate(Timer.getFPGATimestamp() - arm.lastTargetChangeTimestamp).velocity);

    arm.setShoulderSpeed(arm.isLimitShoulder() && shoulderSpeed < 0 ? 0 : shoulderSpeed * -1);
    
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
