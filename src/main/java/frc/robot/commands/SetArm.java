// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
  private Arm arm;
  private armPreset preset;

  public enum armPreset {STOW, LOW, MID, MID_DIAG_TELEOP, MID_DIAG_CUBE, MID_DIAG_CONE, HIGH_CUBE, SHELF};
  
  public SetArm(Arm arm, armPreset preset) {
    this.arm = arm;
    this.preset = preset;
  }

  @Override
  public void initialize() {
    switch (preset) {
      case STOW: {
        arm.elbowTarget = Constants.ELBOW_STOW;
        arm.shoulderTarget = Constants.SHOULDER_STOW;
        break;
      }
      case LOW: {
        arm.elbowTarget = Constants.ELBOW_FLOOR;
        arm.shoulderTarget = Constants.SHOULDER_FLOOR;
        break;
      }
      case MID_DIAG_TELEOP: {
        arm.elbowTarget = Constants.ELBOW_MID_DIAG_TELEOP;
        arm.shoulderTarget = Constants.SHOULDER_MID_DIAG_TELEOP;
        break;
      }
      case MID_DIAG_CONE: {
        arm.elbowTarget = Constants.ELBOW_MID_DIAG_AUTO_CONE;
        arm.shoulderTarget = Constants.SHOULDER_MID_DIAG_AUTO_CONE;
        break;
      }
      case MID_DIAG_CUBE: {
        arm.elbowTarget = Constants.ELBOW_MID_DIAG_AUTO_CUBE;
        arm.shoulderTarget = Constants.SHOULDER_MID_DIAG_AUTO_CUBE;
        break;
      }
      case HIGH_CUBE: {
        arm.elbowTarget = Constants.ELBOW_HIGH_CUBE;
        arm.shoulderTarget = Constants.SHOULDER_HIGH_CUBE;
        break;
      }
      case SHELF: {
        arm.elbowTarget = Constants.ELBOW_SHELF;
        arm.shoulderTarget = Constants.SHOULDER_SHELF;
        break;
      }
    }
  }

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