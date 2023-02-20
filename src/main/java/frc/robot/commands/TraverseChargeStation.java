package frc.robot.commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

public class TraverseChargeStation extends CommandBase {
  private SwerveChassis chassis;
  private double speed;
  
  private double initialRot;
  private int stage;
  private double startTime;
  private double startExtraTime;
 
  public TraverseChargeStation(SwerveChassis chassis, double speed) {
    this.chassis = chassis;
    this.speed = speed;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    initialRot = Math.abs(chassis.getGyroRot(1).getDegrees());
    stage = 0;
  }

  @Override
  public void execute() {
    if (stage < 2) {
      if (Math.abs(chassis.getGyroRot(1).getDegrees()) - initialRot > Constants.THRESHOLD) {
        stage = 1;
      }
      if ((Math.abs(chassis.getGyroRot(1).getDegrees()) - initialRot < Constants.THRESHOLD) && stage == 1) {
        stage = 2;
      }
    } else {
      if (Math.abs(chassis.getGyroRot(1).getDegrees()) + initialRot > Constants.THRESHOLD) {
        stage = 2;
      }
      if ((Math.abs(chassis.getGyroRot(1).getDegrees()) + initialRot < Constants.THRESHOLD) && stage == 2) {
        stage = 3;
        startExtraTime = Timer.getFPGATimestamp();
      }
    }
    chassis.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(speed, 0, 0), chassis.getFusedPose().getRotation()));
    SmartDashboard.putNumber("GyroPitch", Math.abs(chassis.getGyroRot(1).getDegrees()));
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setSpeeds();
  }

  @Override
  public boolean isFinished() {
    return ((Timer.getFPGATimestamp() - startExtraTime > Constants.EXTRA_TIME) && stage == 3) || (Timer.getFPGATimestamp() - startTime > Constants.MAX_TIME);
  }
}