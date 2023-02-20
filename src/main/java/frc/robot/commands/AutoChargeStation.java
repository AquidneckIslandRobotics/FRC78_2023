package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

import javax.swing.JComboBox.KeySelectionManager;

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

  public AutoChargeStation(SwerveChassis chassis, double speed) {
    this.chassis = chassis;
    this.speed = speed;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    initialRot = Math.abs(chassis.getGyroRot(1).getDegrees());
    hasRotated = false;
    hasFlattened = false;
    isReversing = false;
  }

  @Override
  public void execute() {
    if (Math.abs(chassis.getGyroRot(1).getDegrees()) - initialRot > Constants.THRESHOLD) {
      hasRotated = true;
    }
    if ((Math.abs(chassis.getGyroRot(1).getDegrees()) - initialRot < Constants.THRESHOLD) && hasRotated) {
      hasFlattened = true;
    }
    if (hasRotated && hasFlattened && !isReversing) {
      isReversing = true;
      startReverseTime = Timer.getFPGATimestamp();
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