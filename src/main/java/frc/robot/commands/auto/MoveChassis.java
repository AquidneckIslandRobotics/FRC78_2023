// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveChassis;

public class MoveChassis extends CommandBase {
  private SwerveChassis chassis;
  private PIDController xPID;
  private PIDController yPID;
  private PIDController thetaPID;
  private Pose2d targetPose;

  public MoveChassis(SwerveChassis chassis, Pose2d targetPose) {
    this.chassis = chassis;
    this.targetPose = targetPose;
    double p = 1;
    double i = 0;
    double d = 0;
    xPID = new PIDController(p, i, d);
    yPID = new PIDController(p, i, d);
    thetaPID = new PIDController(1, 0, 0);
    xPID.setTolerance(0.1);
    yPID.setTolerance(0.1);
    thetaPID.setTolerance(0.1);
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPID.setSetpoint(targetPose.getX());
    yPID.setSetpoint(targetPose.getY());
    thetaPID.setSetpoint(targetPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = chassis.getFusedPose();
    chassis.setSpeeds(
      new ChassisSpeeds(
      xPID.calculate(currentPose.getX()),
      yPID.calculate(currentPose.getY()),
      thetaPID.calculate(currentPose.getRotation().getRadians()))
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setSpeeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() || yPID.atSetpoint() || thetaPID.atSetpoint();
  }
}
