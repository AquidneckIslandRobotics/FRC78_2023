// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake2024;

public class Set24Intake extends CommandBase {
  Intake2024 intake;
  double topSpeed;
  double bottomSpeed;

  /** Creates a new Set24Intake. */
  public Set24Intake(Intake2024 intake, double topSpeed, double bottomSpeed) {
    this.intake = intake;
    this.topSpeed = topSpeed;
    this.bottomSpeed = bottomSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setTopNeoSpeed(topSpeed);
    intake.setBottomNeoSpeed(bottomSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
