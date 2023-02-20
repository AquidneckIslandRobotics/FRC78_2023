// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dave_Intake;

public class SetFullArm extends CommandBase {
  private Arm arm;
  private Dave_Intake intake;
  private double elbowPos;
  private double shoulderPos;
  private double intakeSpeed;
  private DoubleSolenoid.Value pneumaticState;

  /** Creates a new SetFullArm. */
  public SetFullArm(Arm arm, Dave_Intake intake, double elbowPos, double shoulderPos, double intakeSpeed, DoubleSolenoid.Value pneumaticState) {
    this.arm = arm;
    this.intake = intake;
    this.elbowPos = elbowPos;
    this.shoulderPos = shoulderPos;
    this.intakeSpeed = intakeSpeed;
    this.pneumaticState = pneumaticState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.elbowTarget = elbowPos;
    arm.shoulderTarget = shoulderPos;
    intake.setSpeed(intakeSpeed);
    intake.setSolenoid(pneumaticState);
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
    return Math.abs(arm.shoulderPIDcontroller.getPositionError()) < 2 && Math.abs(arm.elbowPIDcontroller.getPositionError()) < 2;
  }
}