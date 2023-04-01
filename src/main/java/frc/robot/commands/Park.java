// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveChassis;

public class Park extends CommandBase {
  SwerveChassis chassis;
  SwerveModuleState parkingStates[] = {
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
  };
  SwerveModuleState zeroStates[] = {
    new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(0))
  };

  /** Creates a new Park. */
  public Park(SwerveChassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.setStates(parkingStates, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setStates(zeroStates, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}