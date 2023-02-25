package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

public class SwerveDrive extends CommandBase {

  private SwerveChassis chassis;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotSupplier;
  private final IntSupplier dPadSupplier;
  private final DoubleSupplier lTriggerSupplier;
  private final DoubleSupplier rTriggerSupplier;
  private final PIDController thetaPID;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter thetaLimiter;

  public SwerveDrive(
      SwerveChassis chassis,
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier,
      IntSupplier dPadSupplier,
      DoubleSupplier lTriggerSupplier, DoubleSupplier rTriggerSupplier) {
    this.chassis = chassis;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotSupplier = rotSupplier;
    this.dPadSupplier = dPadSupplier;
    this.lTriggerSupplier = lTriggerSupplier;
    this.rTriggerSupplier = rTriggerSupplier;

    xLimiter = new SlewRateLimiter(11, -11, 0);
    yLimiter = new SlewRateLimiter(11, -11, 0);
    thetaLimiter = new SlewRateLimiter(30, -30, 0);
    // thetaPID = new PIDController(5, 0, 0.1);
    thetaPID = new PIDController(5, 0, 0);
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // double dPadX = (dPadSupplier.getAsInt() == 0 ? 1 : 0) - (dPadSupplier.getAsInt() == 180 ? 1 : 0);
    // double dPadY = (dPadSupplier.getAsInt() == 270 ? 1 : 0) - (dPadSupplier.getAsInt() == 90 ? 1 : 0);
    // int invertRelative = Math.abs(chassis.getFusedPose().getRotation().getDegrees()) > 90 ? -1 : 1;
    // int invertRelative = 1;
    // dPadX = triggerAdjust(dPadX) * Constants.DPAD_VEL * invertRelative;
    // dPadY = triggerAdjust(dPadY) * Constants.DPAD_VEL * invertRelative;

    SmartDashboard.putNumber("JoystickX", triggerAdjust(xSupplier.getAsDouble()));
    SmartDashboard.putNumber("JoystickY", triggerAdjust(ySupplier.getAsDouble()));
    SmartDashboard.putNumber("JoystickRot", triggerAdjust(rotSupplier.getAsDouble()));
    thetaPID.setSetpoint(Math.toRadians(dPadSupplier.getAsInt() * -1));
    SmartDashboard.putNumber("DPAD setpoint", thetaPID.getSetpoint());

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        triggerAdjust(xSupplier.getAsDouble()) * Constants.Swerve.MAX_SPEED,
        triggerAdjust(ySupplier.getAsDouble()) * Constants.Swerve.MAX_SPEED,
        triggerAdjust(rotSupplier.getAsDouble()) * Constants.Swerve.MAX_ANGULAR_VELOCITY,
        chassis.getFusedPose().getRotation());

    double currentRot = chassis.getFusedPose().getRotation().getRadians() % (Math.PI * 2);
    double dpadSpeed = dPadSupplier.getAsInt() != -1 ? thetaPID.calculate(currentRot) : 0;
    SmartDashboard.putNumber("DPAD speed", dpadSpeed);
    SmartDashboard.putNumber("DPAD measuredRot", currentRot);
    speeds = new ChassisSpeeds(
      xLimiter.calculate(speeds.vxMetersPerSecond),
      yLimiter.calculate(speeds.vyMetersPerSecond),
      thetaLimiter.calculate(speeds.omegaRadiansPerSecond) + dpadSpeed);
    //with dpad robot relative driving
    // speeds = new ChassisSpeeds(xLimiter.calculate(speeds.vxMetersPerSecond + dPadX),
    //     yLimiter.calculate(speeds.vyMetersPerSecond + dPadY),
    //     thetaLimiter.calculate(speeds.omegaRadiansPerSecond));
    chassis.setSpeeds(speeds, true);
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setSpeeds();
  }

  /**
   * Adjusts the speeds of the given input depending on trigger input, with left
   * trigger decreasing speed and RT increasing
   * 
   * @param in
   * @return Adjusted speed
   */
  public double triggerAdjust(double in) {
    double upAdjust = 0.5;
    double downAdjust = 0.25;
    // Default speed = 1 - upAdjust
    // Full left trigger = 1 - upAdjust - downAdjust
    // Full right trigger = 1
    double triggers = (1 - upAdjust) + (rTriggerSupplier.getAsDouble() * upAdjust)
        - (lTriggerSupplier.getAsDouble() * downAdjust);
    return in * triggers;
  }
}