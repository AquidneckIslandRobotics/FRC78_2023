package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.classes.LimeLight;
// import frc.robot.classes.Odometry;
import frc.robot.classes.SwerveModule;

public class SwerveChassis extends SubsystemBase {

  // abreviations: LU (left upper), RU (right upper), LD (left down), RD (rightdown)
  protected SwerveModule moduleLU, moduleRU, moduleLD, moduleRD;

  protected SwerveDriveKinematics kinematics;
  public SwerveDriveOdometry odometry;
  public SwerveDrivePoseEstimator poseEstimator;
  protected Pigeon2 pidgeon;
  protected LimeLight limelight;

  // KINEMATICS
  protected Translation2d centerOfRot;

  protected ChassisSpeeds speeds = new ChassisSpeeds();

  public SwerveChassis() {
    // WILL COULD CONSIDER MAKING THIS AN ARRAY FOR SIMPLIFIED CODE
    moduleLU = new SwerveModule(0, Constants.Swerve.Mod0.constants);
    moduleRU = new SwerveModule(1, Constants.Swerve.Mod1.constants);
    moduleLD = new SwerveModule(2, Constants.Swerve.Mod2.constants);
    moduleRD = new SwerveModule(3, Constants.Swerve.Mod3.constants);

    pidgeon = new Pigeon2(Constants.PIGEON_IMU);

    centerOfRot = new Translation2d();
    kinematics = Constants.Swerve.SWERVE_KINEMATICS;
    speeds = new ChassisSpeeds();

    limelight = new LimeLight();
    odometry = new SwerveDriveOdometry(kinematics, getGyroRot(), getPositions());

    poseEstimator = new SwerveDrivePoseEstimator(
        getKinematics(),
        getGyroRot(),
        getPositions(),
        new Pose2d(0, 0, getGyroRot().plus(Rotation2d.fromDegrees(90))));

    // CONFGIURE THIS IN A BITs
    // poseEstimator.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02));

    Timer.delay(1.0);
    resetAllToAbsolute();

    resetPose(new Pose2d());
  }

  @Override
  public void periodic() {
    SwerveModule swerveModules[] = { moduleLU, moduleRU, moduleLD, moduleRD };
    for (SwerveModule mod : swerveModules) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }

    poseEstimator.update(getGyroRot(), getPositions());
    Pose2d pose = getFusedPose(); //offset by 8.5, 4.25
    //THIS NEEDS TO BE CONSIDERED WHEN RUNNING AUTONOMOUS
    if(limelight.hasApriltag()){
      PathPlannerServer.sendPathFollowingData(limelight.getBotPose(), new Pose2d(pose.getX(), pose.getY(), pose.getRotation()));
    } else {
      PathPlannerServer.sendPathFollowingData(new Pose2d(), new Pose2d(pose.getX(), pose.getY(), pose.getRotation()));
    }
    
    SmartDashboard.putNumber("FusedPoseX", pose.getX());
    SmartDashboard.putNumber("FusedPoseY", pose.getY());
    SmartDashboard.putNumber("FusedPoseRot", pose.getRotation().getDegrees());
  }

  public void resetPose(Pose2d pose) {
    // Odometry.resetOdometry(pose, getGyroRot(), this, odometry);
    poseEstimator.resetPosition(getGyroRot(), getPositions(), pose);
  }

  public void resetAllToAbsolute() {
    moduleLU.resetToAbsolute();
    moduleRU.resetToAbsolute();
    moduleLD.resetToAbsolute();
    moduleRD.resetToAbsolute();
  }

  // #region GYRO

  public void zeroGyro() {
    pidgeon.setYaw(0.0);
  }
  /**
   * Gets the gyro rotation on any of 3 axises
   * @param axis 0 is Pitch (X), 1 is Roll (Y), 2 is Yaw (Z)
   * @return
   */
  public Rotation2d getGyroRot(int axis) {
    if(axis == 0) {
      return Rotation2d.fromDegrees(pidgeon.getPitch());
    }
    if(axis == 1) {
      return Rotation2d.fromDegrees(pidgeon.getRoll());
    }
    if(axis == 2) {
      return Rotation2d.fromDegrees(pidgeon.getYaw());
    }
    return new Rotation2d();
  }
  /**
   * Returns gyro rotation on Yaw axis (Z)
   * @return
   */
  public Rotation2d getGyroRot() {
    return getGyroRot(2);
  }

  // #endregion
  // #region GET FUNCTIONS

  public Pose2d getFusedPose() {
    if (limelight.hasApriltag()) {
      Pose2d pose = limelight.getBotPose();
      poseEstimator.addVisionMeasurement(pose, limelight.getBotPoseTimestamp());
      poseEstimator.resetPosition(getGyroRot().plus(Rotation2d.fromDegrees(0)), getPositions(), poseEstimator.getEstimatedPosition());
    }
    return poseEstimator.getEstimatedPosition();
}

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition positions[] = {
        moduleLU.getPosition(),
        moduleRU.getPosition(),
        moduleLD.getPosition(),
        moduleRD.getPosition(),
    };
    return positions;
  }

  // #endregion
  // #region SET FUNCTIONS

  public void speedsToStates(Boolean isOpenLoop) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, centerOfRot);
    setStates(states, isOpenLoop, false);
  }

  public void setCenter(Translation2d translation) {
    centerOfRot = translation;
  }
  /**
   * Sets chassis speeds, with open loop and without calling speedsToStates
   * @param speeds
   */
  public void setSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
    this.speeds = speeds;
    SmartDashboard.putNumber("ChassisSpeedsX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeedsY", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeedsRot", speeds.omegaRadiansPerSecond);

    // the the teleop periodic in Robot it calls speeds to states
    if(!isOpenLoop){
      speedsToStates(isOpenLoop);
    }
  }

  /** Overload for setSpeeds, sets speeds to 0, 0, 0 */
  public void setSpeeds() {
    setSpeeds(new ChassisSpeeds(0, 0, 0), false);
  }

  /**
   * Sets chassis speeds, but with a closed loop PID
   * @param speeds
   */
  public void setSpeeds(ChassisSpeeds speeds) {
    setSpeeds(speeds, false);
  }

  /**
   * Sets the state of each module
   * 
   * @param states     Must be exactly of length 4
   * @param isOpenLoop
   */
  public void setStates(SwerveModuleState[] states, boolean isOpenLoop, boolean overrideDeadband) {
    if (states.length != 4) {
      throw new IllegalArgumentException("The \"setStates\" input array size should be 4!");
    } else {
      // Odometry.updateOdometry(getPositions(), getGyroRot(), odometry);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_SPEED);
      moduleLU.setDesiredState(states[0], isOpenLoop, overrideDeadband);
      moduleRU.setDesiredState(states[1], isOpenLoop, overrideDeadband);
      moduleLD.setDesiredState(states[2], isOpenLoop, overrideDeadband);
      moduleRD.setDesiredState(states[3], isOpenLoop, overrideDeadband);
    }
  }
  // #endregion
}