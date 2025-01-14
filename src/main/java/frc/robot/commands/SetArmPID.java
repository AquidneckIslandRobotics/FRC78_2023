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

  private TrapezoidProfile elbowProfile;  
  private TrapezoidProfile shoulderProfile;
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
    elbowProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(90,60), elbow_goal, elbow_currentPos);//185 120
    shoulderProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(80, 40), shoulder_goal, shoulder_currentPos); //165 80
    calcElbowGoal = elbowProfile.calculate(Timer.getFPGATimestamp() - arm.lastTargetChangeTimestamp).position;
    calcShoulderGoal = shoulderProfile.calculate(Timer.getFPGATimestamp() - arm.lastTargetChangeTimestamp).position;
    SmartDashboard.putNumber("TrapElbowGoal", elbowProfile.calculate(Timer.getFPGATimestamp() - arm.lastTargetChangeTimestamp).velocity);
    SmartDashboard.putNumber("TrapShoulderGoal", shoulderProfile.calculate(Timer.getFPGATimestamp() - arm.lastTargetChangeTimestamp).velocity);
    double shoulderSpeed = arm.shoulderPIDcontroller.calculate(arm.getShoulderAbsolutePosition(), arm.shoulderTarget); 

    // if (shoulderSpeed < 0){
    //   shoulderSpeed = shoulderSpeed * 0.15;
    // }
    arm.setShoulderSpeed(arm.isLimitShoulder() && shoulderSpeed < 0 ? 0 : shoulderSpeed * -1);
    
    double elbowSpeed = arm.elbowPIDcontroller.calculate(arm.getElbowAbsolutePosition(), arm.elbowTarget);
    // if (elbowSpeed > 0){
    //   elbowSpeed = elbowSpeed * 0.55;
    // }
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
