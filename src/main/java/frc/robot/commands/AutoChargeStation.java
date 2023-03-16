package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

import org.opencv.core.Mat.Tuple3;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class AutoChargeStation extends CommandBase {
  private SwerveChassis chassis;
  private double speed;
  
  private double initialRot;
  private double startTime;
  private double startReverseTime;
  private double startWaitTime;
  private stage currentStage;
  //BELOW: below threshold, hasn't started climbing yet
  //ABOVE: when above the threshold; started climbing
  //AFTER: after passing back below the threshold
  //REVERSE: when reversing, waiting for reverse timer to end
  //WAIT: when waiting after reveresing before knowing if it has to correct
  //CORRECT: final correction using logic
  //DONE: when set to done, command stops
  private enum stage {BELOW, ABOVE, AFTER, REVERSE, WAIT, CORRECT, DONE}

  public AutoChargeStation(SwerveChassis chassis, double speed) {
    this.chassis = chassis;
    this.speed = speed;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    initialRot = Math.abs(chassis.getGyroRot(1).getDegrees());
    currentStage = stage.BELOW;
  }

  @Override
  public void execute() {
    double inclination = chassis.getGyroRot(1).getDegrees();

    switch (currentStage) {
      case BELOW: {
        if (Math.abs(inclination) - initialRot > Constants.THRESHOLD) {
          currentStage = stage.ABOVE;
        }
        SmartDashboard.putString("AutoChargeStatus", "BELOW");
      }
      case ABOVE: {
        if (Math.abs(inclination) - initialRot < Constants.THRESHOLD) {
          currentStage = stage.AFTER;
        }
        SmartDashboard.putString("AutoChargeStatus", "ABOVE");
      }
      case AFTER: {
        startReverseTime = Timer.getFPGATimestamp();
        currentStage = stage.REVERSE;
        SmartDashboard.putString("AutoChargeStatus", "AFTER");
      }
      case REVERSE: {
        if (Timer.getFPGATimestamp() - startReverseTime > Constants.REVERSE_TIME) {
          startReverseTime = Timer.getFPGATimestamp();
          currentStage = stage.WAIT;
          SmartDashboard.putString("AutoChargeStatus", "REVERSE");
        }
      }
      case WAIT: {
        if (Timer.getFPGATimestamp() - startWaitTime > 0.5) { // TODO
          if (Math.abs(inclination) - initialRot < 3) { // we could just switch to correct, but that would take an extra period
            currentStage = stage.DONE;
          } else {
            currentStage = stage.CORRECT;
          }
        }
        SmartDashboard.putString("AutoChargeStatus", "WAITING");
      }
      case CORRECT: {
        if (Math.abs(inclination) - initialRot < 3) { // TODO
          currentStage = stage.DONE;
        }
        SmartDashboard.putString("AutoChargeStatus", "CORRECT");
      }
    }
    
    if (currentStage == stage.BELOW || currentStage == stage.ABOVE) {
      chassis.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(speed, 0, 0), chassis.getFusedPose().getRotation()));
    } else if (currentStage == stage.REVERSE) {
      chassis.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-Math.signum(speed) * Constants.REVERSE_SPEED, 0, 0), chassis.getFusedPose().getRotation()));
    } else if (currentStage == stage.CORRECT) {
      chassis.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(Math.signum(speed) * 0.5 * inclination < 0 ? 1 : -1, 0, 0), chassis.getFusedPose().getRotation()));
    }
    SmartDashboard.putNumber("GyroPitch", Math.abs(chassis.getGyroRot(1).getDegrees()) - initialRot);
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setSpeeds();
    SmartDashboard.putString("AutoChargeStatus", "DONE");
  }

  @Override
  public boolean isFinished() {
    return currentStage == stage.DONE || (Timer.getFPGATimestamp() - startTime > Constants.MAX_TIME);
  }
}