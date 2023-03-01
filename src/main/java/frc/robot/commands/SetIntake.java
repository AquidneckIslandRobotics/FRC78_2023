// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dave_Intake;

public class SetIntake extends CommandBase {
  Dave_Intake intake;
  double speed;
  DoubleSolenoid.Value pieceSolenoidValue;
  DoubleSolenoid.Value dropSolenoidValue;

  public SetIntake(Dave_Intake intake, DoubleSolenoid.Value dropSolenoidValue, DoubleSolenoid.Value pieceSolenoidValue, double speed) {
    this.intake = intake;
    this.speed = speed;
    this.pieceSolenoidValue = pieceSolenoidValue;
    this.dropSolenoidValue = dropSolenoidValue;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setSpeed(speed);
    intake.setSolenoidPiece(pieceSolenoidValue);
    intake.setSolenoidDrop(dropSolenoidValue);
  }

  @Override
  public void end(boolean interrupted) { }

  @Override
  public boolean isFinished() {
    return true;
  }
}