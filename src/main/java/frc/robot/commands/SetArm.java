// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
  private double elbowTarget;
  private double shoulderTarget;
  public enum armPreset {STOW, LOW, MID, MID_DIAG_TELEOP, MID_DIAG_CUBE, MID_DIAG_CONE, HIGH_CUBE, SHELF};
  private armPreset preset;

  private Arm arm;

  public SetArm(Arm arm, armPreset preset) {
    this.arm = arm;
    this.preset = preset;
  }

  @Override
  public void initialize() {
    switch (preset) {
      case STOW: {
        elbowTarget = Constants.ELBOW_STOW;
        shoulderTarget = Constants.SHOULDER_STOW;
        break;
      }
      case LOW: {
        elbowTarget = Constants.ELBOW_FLOOR;
        shoulderTarget = Constants.SHOULDER_FLOOR;
        break;
      }
      case MID_DIAG_TELEOP: {
        elbowTarget = Constants.ELBOW_MID_DIAG_TELEOP;
        shoulderTarget = Constants.SHOULDER_MID_DIAG_TELEOP;
        break;
      }
      case MID_DIAG_CONE: {
        elbowTarget = Constants.ELBOW_MID_DIAG_AUTO_CONE;
        shoulderTarget = Constants.SHOULDER_MID_DIAG_AUTO_CONE;
        break;
      }
      case MID_DIAG_CUBE: {
        elbowTarget = Constants.ELBOW_MID_DIAG_AUTO_CUBE;
        shoulderTarget = Constants.SHOULDER_MID_DIAG_AUTO_CUBE;
        break;
      }
      case HIGH_CUBE: {
        elbowTarget = Constants.ELBOW_HIGH_CUBE;
        shoulderTarget = Constants.SHOULDER_HIGH_CUBE;
        break;
      }
      case SHELF: {
        elbowTarget = Constants.ELBOW_SHELF;
        shoulderTarget = Constants.SHOULDER_SHELF;
        break;
      }
    }
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