package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

import org.opencv.core.Mat.Tuple3;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoChargeStation extends CommandBase {
  private SwerveChassis chassis;
  private double speed;
  
  private double initialRot;
  private boolean hasRotated;
  private boolean hasFlattened;
  private boolean isReversing;
  private double startTime;
  private double startReverseTime;
  private stage currentStage;

  private enum stage {BELOW, ABOVE, AFTER, REVERSE, CORRECT}

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
    hasRotated = false;
    hasFlattened = false;
    isReversing = false;
  }

  @Override
  public void execute() {
    switch (currentStage) {
      case BELOW: {
        if (Math.abs(chassis.getGyroRot(1).getDegrees()) - initialRot > Constants.THRESHOLD) {
          currentStage = stage.ABOVE;
        }
      }
      case ABOVE: {
        if (Math.abs(chassis.getGyroRot(1).getDegrees()) - initialRot < Constants.THRESHOLD) {
          currentStage = stage.AFTER;
        }
      }
      case AFTER: {
        startReverseTime = Timer.getFPGATimestamp();
      }
    }
    
    if (hasRotated && hasFlattened && !isReversing) {
      isReversing = true;
    }
    if (!isReversing) {
      chassis.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(speed, 0, 0), chassis.getFusedPose().getRotation()));
    } else {
      chassis.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-Math.signum(speed) * Constants.REVERSE_SPEED, 0, 0), chassis.getFusedPose().getRotation()));
    }
    SmartDashboard.putNumber("GyroPitch", Math.abs(chassis.getGyroRot(1).getDegrees()) - initialRot);
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setSpeeds();
  }

  @Override
  public boolean isFinished() {
    return ((Timer.getFPGATimestamp() - startReverseTime > Constants.REVERSE_TIME) && isReversing) || (Timer.getFPGATimestamp() - startTime > Constants.MAX_TIME);
  }
}