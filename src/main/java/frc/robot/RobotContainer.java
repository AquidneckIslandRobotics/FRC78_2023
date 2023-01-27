package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.classes.Odometry;
import frc.robot.classes.PathFunctions;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.IntakeV1_Lentz;
import frc.robot.subsystems.SwerveChassis;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
public class RobotContainer {

  public final SwerveChassis m_chassis;
  private final XboxController m_driveController;
  //private final XboxController manipControl;
  private final IntakeV1_Lentz m_IntakeV1_Lentz = new IntakeV1_Lentz();

  private final HashMap<String, Command> m_eventMap;
  private final SwerveAutoBuilder autoBuilder;
 private final CommandJoystick manipControl = new CommandJoystick(0);

 Trigger btnX = manipControl.button(1);
  Trigger btnY = manipControl.button(2);
  Trigger btnA = manipControl.button(3);
  Trigger btnB = manipControl.button(4);

  public RobotContainer() {
    m_chassis = new SwerveChassis();

    m_driveController = new XboxController(Constants.driverController);
    //m_manipControl = new XboxController(0);
    //private final CommandJoystick manipControl = new CommandJoystick(0);
   // private final CommandJoystick manipControl = new CommandJoystick(0);
    m_chassis.setDefaultCommand(new SwerveDrive(
        m_chassis,
        () -> -modifyAxis(m_driveController.getLeftY()),
        () -> -modifyAxis(m_driveController.getLeftX()),
        () -> -modifyAxis(m_driveController.getRightX()),
        () -> m_driveController.getPOV(),
        () -> m_driveController.getLeftTriggerAxis(),
        () -> m_driveController.getRightTriggerAxis()));

    // #region PATHPLANNER
    m_eventMap = new HashMap<>();
    m_eventMap.put("Waypoint1Reached", new PrintCommand("Waypoint 1 reached!"));
    m_eventMap.put("command1", new PrintCommand("Hello World"));

    // An object used to do much of the creating path following commands
    autoBuilder = new SwerveAutoBuilder(
        m_chassis::getPose,
        m_chassis::resetPose,
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(0.5, 0.0, 0.0),
        m_chassis::setSpeedsAuto,
        m_eventMap,
        m_chassis);

    PathPlannerServer.startServer(5811);
    // #endregion

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new Trigger(m_driveController::getYButton).onTrue(new InstantCommand(m_chassis::zeroGyro));// (new
                                                                                               // InstantCommand(m_chassis::zeroGyro));
    SwerveModuleState[] emptyStates = {
        new SwerveModuleState(0.01, new Rotation2d()),
        new SwerveModuleState(0.01, new Rotation2d()),
        new SwerveModuleState(0.01, new Rotation2d()),
        new SwerveModuleState(0.01, new Rotation2d())
    };
    new Trigger(m_driveController::getXButton).onTrue(new InstantCommand(() -> m_chassis.setStates(emptyStates, true)));
    new Trigger(m_driveController::getBButton).onTrue(new InstantCommand(() -> m_chassis.resetAllToAbsolute()));
    new Trigger(m_driveController::getRightBumper)
        .onTrue(new InstantCommand(() -> m_chassis.setCenter(new Translation2d(1, 0))));
    new Trigger(m_driveController::getRightBumper)
        .onFalse(new InstantCommand(() -> m_chassis.setCenter(new Translation2d(0, 0))));

    //Intake Buttons for V1 

        btnX.onTrue(m_IntakeV1_Lentz.runTopNeo(0.1)).onFalse(m_IntakeV1_Lentz.runTopNeo(0));
   
   
        btnY.onTrue(m_IntakeV1_Lentz.runTopNeo(-0.1)).onFalse(m_IntakeV1_Lentz.runTopNeo(0));
     
        btnA.onTrue(m_IntakeV1_Lentz.runBottomNeo(1)).onFalse(m_IntakeV1_Lentz.runBottomNeo(0));
     
        btnB.onTrue(m_IntakeV1_Lentz.runBottomNeo(-0.50)).onFalse(m_IntakeV1_Lentz.runBottomNeo(0.0));

    //End of Intake buttons for V1




    
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
    value = Math.copySign(value * value, value);
    return value;
  }
}