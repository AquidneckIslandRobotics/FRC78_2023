package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.classes.LimeLight;
import frc.robot.classes.Odometry;
import frc.robot.classes.PathFunctions;
import frc.robot.commands.ArmControl;
import frc.robot.commands.AutoCenter;
import frc.robot.commands.Park;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.TestShoulderMotor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveChassis;
//import frc.robot.commands.ManualControl;
import frc.robot.commands.RunArmToTarget;

public class RobotContainer {

  public final SwerveChassis m_chassis;
  public final Arm m_arm;
  private final LimeLight m_limeLight;
  
  private final XboxController m_driveController;
  private final XboxController m_armController;

  private final HashMap<String, Command> m_eventMap;
  private final SwerveAutoBuilder autoBuilder;

  public RobotContainer() {
    m_chassis = new SwerveChassis();
    m_arm = new Arm();
    m_limeLight = new LimeLight();
    m_driveController = new XboxController(Constants.DRIVE_CONTROLLER);

    m_armController = new XboxController(Constants.ARM_Controller);

    m_chassis.setDefaultCommand(new SwerveDrive(
        m_chassis,
        () -> -modifyAxis(m_driveController.getLeftY()),
        () -> -modifyAxis(m_driveController.getLeftX()),
        () -> -modifyAxis(m_driveController.getRightX()),
        () -> m_driveController.getPOV(),
        () -> modifyAxis(m_driveController.getLeftTriggerAxis()),
        () -> modifyAxis(m_driveController.getRightTriggerAxis())));

      //m_arm.setDefaultCommand(new ManualControl(m_arm, m_armController.getRightY(), m_armController.getLeftY()));
     // m_arm.setDefaultCommand(new ArmControl(m_arm, m_armController.getLeftY(), m_armController.getRightY()));
      m_arm.setDefaultCommand(new ArmControl(m_arm,
      () -> -modifyAxis(m_armController.getLeftY()),
      () -> -modifyAxis(m_armController.getRightY())
      ));
    //  m_arm.setDefaultCommand(new TestShoulderMotor(m_arm));
    
  //  m_arm.setDefaultCommand(new InstantCommand(()-> m_arm.setShoulderSpeed(0.2)));//will change-MG

    
    Trigger buttonA = new JoystickButton(m_armController, XboxController.Button.kX.value);
    buttonA.onTrue(new InstantCommand(() -> new RunArmToTarget(m_arm, Constants.shoulderLowTarget, Constants.elbowLowTarget)));
    buttonA.onFalse(new InstantCommand(() -> m_arm.setShoulderSpeed(0)));
    
    Trigger buttonB = new JoystickButton(m_armController, XboxController.Button.kX.value);
    buttonB.onTrue(new InstantCommand(() -> new RunArmToTarget(m_arm, Constants.shoulderMidTarget, Constants.elbowMidTarget)));
    buttonB.onFalse(new InstantCommand(() -> m_arm.setShoulderSpeed(0)));

    
    
    //Trigger buttonY = new JoystickButton(m_armController, XboxController.Button.kY.value);
   // buttonY.onTrue(new InstantCommand(() -> m_arm.setElbowSpeed(0.5)));
    //buttonY.onFalse(new InstantCommand(() -> m_arm.setElbowSpeed(0)));

   //Trigger buttonB = new JoystickButton(m_armController, XboxController.Button.kY.value);
    //buttonB.onTrue(new InstantCommand(() -> m_arm.setElbowSpeed(-0.5)));
    //buttonB.onFalse(new InstantCommand(() -> m_arm.setElbowSpeed(0)));

    // #region PATHPLANNER
    m_eventMap = new HashMap<>();
    m_eventMap.put("Waypoint1Reached", new PrintCommand("Waypoint 1 reached!"));
    m_eventMap.put("command1", new PrintCommand("Hello World"));
    m_eventMap.put("Park", new Park(m_chassis));

    // An object used to do much of the creating path following commands
    autoBuilder = new SwerveAutoBuilder(
        m_chassis::getPose,
        m_chassis::resetPose,
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(0.5, 0.0, 0.0),
        m_chassis::setSpeeds,
        m_eventMap,
        m_chassis);

    PathPlannerServer.startServer(5811);
    // #endregion

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new Trigger(m_driveController::getStartButton).onTrue(new InstantCommand(m_chassis::zeroGyro));// (new
                                                                                               // InstantCommand(m_chassis::zeroGyro));
    new Trigger(m_driveController::getYButton).whileTrue(new AutoCenter(m_limeLight, new Pose2d(-1.2, 0, new Rotation2d(0)), m_chassis));
    new Trigger(m_driveController::getXButton).whileTrue(new AutoCenter(m_limeLight, new Pose2d(-1.2, 0.5, new Rotation2d(0)), m_chassis));
    new Trigger(m_driveController::getBButton).whileTrue(new AutoCenter(m_limeLight, new Pose2d(-1.2, -0.5, new Rotation2d(0)), m_chassis));
    new Trigger(m_driveController::getAButton).onTrue(new InstantCommand(() -> m_chassis.resetAllToAbsolute()));
    new Trigger(m_driveController::getRightBumper)
        .onTrue(new InstantCommand(() -> m_chassis.setCenter(new Translation2d(1, 0))));
    new Trigger(m_driveController::getRightBumper)
        .onFalse(new InstantCommand(() -> m_chassis.setCenter(new Translation2d(0, 0))));
    new Trigger(m_driveController::getBackButton).whileTrue(new Park(m_chassis));

  }

  //value * max * joystickY

  public Command getAutonomousCommand() {
    PathPlannerTrajectory trajectory1 = PathFunctions.createTrajectory("Test3");

    Odometry.resetOdometry(trajectory1.getInitialHolonomicPose(), m_chassis.getGyroRot(), m_chassis, m_chassis.odometry);
    return autoBuilder.followPath(trajectory1).andThen(() -> m_chassis.setSpeeds());
  }

  /**
   * Applies a deadband to the given joystick axis value
   * 
   * @param value
   * @param deadband
   * @return
   */
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      return (value > 0.0 ? value - deadband : value + deadband) / (1.0 - deadband);
    } else {
      return 0.0;
    }
  }

  /**
   * Processes the given joystick axis value, applying deadband and squaring it
   * 
   * @param value
   * @return
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);
    // Square the axis
    // value = Math.copySign(value * value, value);
    return value;
  }
}