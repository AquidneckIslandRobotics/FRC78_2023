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
  private double startWaitTime;
  private stage currentStage;
  //BELOW: below threshold, hasn't started climbing yet
  //ABOVE: when above the threshold; started climbing
  //WAIT: when waiting after reveresing before knowing if it has to correct
  //CORRECT: final correction using logic
  //DONE: when set to done, command stops
  private enum stage {BELOW, ABOVE, WAIT, CORRECT, DONE}

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
    double deltaInclination = Math.abs(inclination) - initialRot;
    SmartDashboard.putNumber("inclination", inclination);
    SmartDashboard.putNumber("deltaInclination", deltaInclination);

    switch (currentStage) {
      case BELOW: {
        if (deltaInclination > Constants.THRESHOLD) {
          currentStage = stage.ABOVE;
        }
        SmartDashboard.putString("AutoChargeStatus", "BELOW");
        break;
      }
      case ABOVE: {
        if (deltaInclination < Constants.THRESHOLD) {
          startWaitTime = Timer.getFPGATimestamp();
          currentStage = stage.WAIT;
        }
        SmartDashboard.putString("AutoChargeStatus", "ABOVE");
        break;
      }
      case WAIT: {
        if (Timer.getFPGATimestamp() - startWaitTime > Constants.WAIT_TIME) {
          if (deltaInclination < 3) { // we could just switch to correct, but that would take an extra cycle
            currentStage = stage.DONE;
          } else {
            currentStage = stage.CORRECT;
          }
        }
        SmartDashboard.putString("AutoChargeStatus", "WAITING");
        break;
      }
      case CORRECT: {
        if (deltaInclination < Constants.CORRECT_THRES) {
          currentStage = stage.DONE;
        }
        SmartDashboard.putString("AutoChargeStatus", "CORRECT");
        break;
      }
    }
    
    if (currentStage == stage.BELOW || currentStage == stage.ABOVE) {
      chassis.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
        speed + (-Math.signum(speed) * Constants.CLIMBING_VEL_FACTOR * (Math.min(Constants.THRESHOLD, deltaInclination) / Constants.THRESHOLD)), 
        0, 0), chassis.getFusedPose().getRotation()));
    } else if (currentStage == stage.CORRECT) {
      chassis.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(Math.signum(speed) * Constants.CORRECT_VEL * inclination < 0 ? -1 : 1, 0, 0), chassis.getFusedPose().getRotation()));
    }
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