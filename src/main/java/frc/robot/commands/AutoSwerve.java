// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveChassis;

public class AutoSwerve extends CommandBase {
  private SwerveChassis chassis;
  private double startTime;

  private static final double revereseTime = 3;
  private static final double speed = 1;
  private static final double reverseSpeed = -0.7;
  /** Creates a new AutoSwerve. */
  public AutoSwerve(SwerveChassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    chassis.setSpeeds(new ChassisSpeeds(speed, 0, 0));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     chassis.setSpeeds(new ChassisSpeeds(reverseSpeed, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setSpeeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Timer.getFPGATimestamp() - startTime > revereseTime));
  }
}
