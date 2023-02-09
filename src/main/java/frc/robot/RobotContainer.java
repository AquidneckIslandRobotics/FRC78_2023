  package frc.robot;

import java.time.Instant;
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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.classes.LimeLight;
import frc.robot.classes.Odometry;
import frc.robot.classes.PathFunctions;

import frc.robot.commands.ArmControl;
import frc.robot.commands.AutoCenter;
import frc.robot.commands.Park;
import frc.robot.commands.ReverseBottomNeo;
import frc.robot.commands.ReverseTopNeo;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.TestShoulderMotor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveChassis;
//import frc.robot.commands.ManualControl;
import frc.robot.commands.SetArm;
import frc.robot.commands.RunBottomNeos;
import frc.robot.commands.RunTopBottomTemp;
import frc.robot.commands.RunTopNeos;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Dave_Intake;
import frc.robot.subsystems.IntakeV1_Lentz;
import frc.robot.subsystems.SwerveChassis;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;


public class RobotContainer {

  public final SwerveChassis m_chassis;
  public final Arm m_arm;
  private final LimeLight m_limeLight;
  
  private final XboxController m_driveController;
  private final XboxController m_manipController;
  private final IntakeV1_Lentz m_IntakeV1_Lentz;
  private final Dave_Intake m_Dave_Intake;
  private final HashMap<String, Command> m_eventMap;
  private final SwerveAutoBuilder autoBuilder;

  // Trigger btnX = manipControl.x(null);
  // Trigger btnY = manipControl.y.value;
  // Trigger btnA = manipControl.a.value;
  // Trigger btnB = manipControl.b.value;

  public RobotContainer() {
    m_chassis = new SwerveChassis();
    m_arm = new Arm();
    m_limeLight = new LimeLight();
    m_driveController = new XboxController(Constants.DRIVE_CONTROLLER);

    m_IntakeV1_Lentz = new IntakeV1_Lentz();


    m_Dave_Intake = new Dave_Intake();
    m_manipController = new XboxController(Constants.MANIP_CONTROLLER);

    m_chassis.setDefaultCommand(new SwerveDrive(
        m_chassis,
        () -> -modifyAxis(m_driveController.getLeftY()),
        () -> -modifyAxis(m_driveController.getLeftX()),
        () -> -modifyAxis(m_driveController.getRightX()),
        () -> m_driveController.getPOV(),
        () -> modifyAxis(m_driveController.getLeftTriggerAxis()),
        () -> modifyAxis(m_driveController.getRightTriggerAxis())));

      m_arm.setDefaultCommand(new ArmControl(m_arm,
      () -> -modifyAxis(m_manipController.getLeftY()),
      () -> -modifyAxis(m_manipController.getRightY())
      ));

  m_Dave_Intake.setDefaultCommand(new SetIntake(m_Dave_Intake, 0.1, DoubleSolenoid.Value.kForward));

    Trigger buttonA = new JoystickButton(m_manipController, XboxController.Button.kX.value);
    buttonA.onTrue(new InstantCommand(() -> new SetArm(m_arm, Constants.SHOULDER_LOW_TARGET, Constants.ELBOW_LOW_TARGET)));
    buttonA.onFalse(new InstantCommand(() -> m_arm.setShoulderSpeed(0)));
    
    Trigger buttonB = new JoystickButton(m_manipController, XboxController.Button.kX.value);
    buttonB.onTrue(new InstantCommand(() -> new SetArm(m_arm, Constants.SHOULDER_MID_TARGET, Constants.ELBOW_MID_TARGET)));
    buttonB.onFalse(new InstantCommand(() -> m_arm.setShoulderSpeed(0)));

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



    //Intake Buttons for V1 
    // new Trigger(m_manipController::getXButton).onTrue(m_IntakeV1_Lentz.runTopNeo(0.5)).onFalse((m_IntakeV1_Lentz.runTopNeo(0)));
    // new Trigger(m_manipController::getYButton).onTrue(m_IntakeV1_Lentz.runTopNeo(-0.5)).onFalse((m_IntakeV1_Lentz.runTopNeo(0)));
    // new Trigger(m_manipController::getAButton).onTrue(m_IntakeV1_Lentz.runBottomNeo(0.5)).onFalse((m_IntakeV1_Lentz.runTopNeo(0)));
    // new Trigger(m_manipController::getXButton).onTrue(m_IntakeV1_Lentz.runBottomNeo(-0.50)).onFalse((m_IntakeV1_Lentz.runTopNeo(0)));
    //new Trigger(m_manipController::getAButton).onTrue(new InstantCommand(() -> m_IntakeV1_Lentz.testNeo())).onFalse(new InstantCommand(() -> m_IntakeV1_Lentz.stopNeo()));
    //new Trigger(m_manipController::getAButton).whileTrue(new RunTopBottomTemp(m_IntakeV1_Lentz));
    new Trigger(m_manipController::getAButton).whileTrue(new RunTopNeos(m_IntakeV1_Lentz, -0.6));
    new Trigger(m_manipController::getBButton).whileTrue(new RunBottomNeos(m_IntakeV1_Lentz, -0.3));
    new Trigger(m_manipController::getXButton).whileTrue(new RunTopNeos(m_IntakeV1_Lentz, 0.6));
    new Trigger(m_manipController::getYButton).whileTrue(new RunBottomNeos(m_IntakeV1_Lentz, 0.3));

   new Trigger(m_manipController::getAButton).whileTrue(new SetArm(m_arm, 213.211, 123.627));
   new Trigger(m_manipController::getBButton).whileTrue(new SetArm(m_arm, 231.899, 119.261));
   //new Trigger(m_manipController::getXButton).whileTrue(new SetArm(m_arm, 0, 0));
   new Trigger(m_manipController::getYButton).whileTrue(new SetArm(m_arm, 115.974, 24.974 ));

    //End of Intake buttons for V1
    // Intake buttons for Dave's intake (X = intake)

   new Trigger(m_manipController::getXButton).whileTrue(new SetIntake(m_Dave_Intake, 0.6, DoubleSolenoid.Value.kForward)); 
   new Trigger(m_manipController::getYButton).whileTrue(new SetIntake(m_Dave_Intake, 0.6, DoubleSolenoid.Value.kReverse)); 
   new Trigger(m_manipController::getAButton).whileTrue(new SetIntake(m_Dave_Intake, -1, DoubleSolenoid.Value.kReverse));
  }

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