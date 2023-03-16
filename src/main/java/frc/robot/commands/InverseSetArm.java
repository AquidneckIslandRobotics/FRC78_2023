// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class InverseSetArm extends CommandBase {
  private double x;
  private double y;
  private Arm arm;
  
  
  /** Creates a new RunArmToTarget. */
  public InverseSetArm(Arm arm, double xTarget, double yTarget) {
    this.arm = arm;
    this.x = xTarget;
    this.y = yTarget;
  }
  
  @Override
  public void initialize() {
    
      // Define arm lengths in inches
      double L1 = 15.0; //the length of the arm segment between the shoulder joint and the elbow joint
      double L2 = 15.0; //the length of the second arm, which is the distance between the elbow joint and the end-effector of the arm 
    
      // Calculate the distance from the base of the arm to the point (x, y)
      double r = Math.sqrt(x*x + y*y);
    
      // Calculate the angle between the arm and the x-axis
      double theta1 = Math.atan2(y, x);
    
      // Calculate the angle between the upper arm and the horizontal
      double cosTheta2 = (r*r - L1*L1 - L2*L2) / (2*L1*L2); //calculates cosine of the angle
      double sinTheta2 = Math.sqrt(1 - cosTheta2*cosTheta2); //sine of the angle
      double theta2 = Math.atan2(sinTheta2, cosTheta2);  //angle itself   
    
      // Calculate the joint angles
      double shoulderAngle = theta1 - Math.atan2(L2*sinTheta2, L1 + L2*cosTheta2); //The first line calculates the angle between the base of the arm and the shoulder joint
      double elbowAngle = Math.PI - theta2; //the second line calculates the angle between the upper and lower arms at the elbow joint.
    
    arm.elbowTarget = elbowAngle;
    arm.shoulderTarget = shoulderAngle;
  }

  @Override
  public void execute() { }

  @Override
  public void end(boolean interrupted) {
    System.out.print("SetArm command ended!");
    arm.shoulderPIDcontroller.reset();
    arm.elbowPIDcontroller.reset();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(arm.shoulderPIDcontroller.getPositionError()) < 2 && Math.abs(arm.elbowPIDcontroller.getPositionError()) < 2;
  }
}