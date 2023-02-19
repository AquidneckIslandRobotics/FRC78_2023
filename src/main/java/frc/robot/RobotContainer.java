  package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.classes.LimeLight;
import frc.robot.classes.PathFunctions;
import frc.robot.commands.ArmControl;
import frc.robot.commands.AutoCenter;
import frc.robot.commands.AutoChargeStation;
import frc.robot.commands.Park;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetArm;
import frc.robot.commands.SetArmPID;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dave_Intake;
import frc.robot.subsystems.SwerveChassis;


public class RobotContainer {

  public final SwerveChassis m_chassis;
  public final Arm m_arm;
  private final LimeLight m_limeLight;
  private final UsbCamera m_driverCam;
  public final MjpegServer m_mjpegServer;
  
  private final XboxController m_driveController;
  private final XboxController m_manipController;
  //private final IntakeV1_Lentz m_IntakeV1_Lentz;
  private final Dave_Intake m_Dave_Intake;
  private final HashMap<String, Command> m_eventMap;
  private final SwerveAutoBuilder autoBuilder;

  static enum AUTOS {EMPTY, SIX_TAXI, SEVEN_CHARGE, SIX_CONE_TAXI, SEVEN_CONE_CHARGE};
  public SendableChooser<AUTOS> firstAutoCmd = new SendableChooser<>();
  // private SendableChooser<Command> secondAutoCmd = new SendableChooser();
  // private SendableChooser<Command> thirdAutoCmd = new SendableChooser();

  public RobotContainer() {
    m_chassis = new SwerveChassis();
    m_arm = new Arm();
    m_limeLight = new LimeLight();
    m_driveController = new XboxController(Constants.DRIVE_CONTROLLER);

    //m_IntakeV1_Lentz = new IntakeV1_Lentz();

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
   m_arm.setDefaultCommand(new SetArmPID(m_arm));

   m_driverCam = CameraServer.startAutomaticCapture();
   m_driverCam.setResolution(256, 192);
  //  m_driverCam.setResolution(64, 43);
   m_mjpegServer = new MjpegServer("driverCamServer", 1181);
   m_mjpegServer.setSource(m_driverCam);
    //  CvSink cvSink = CameraServer.getVideo();
    //  CvSource outputStream = CameraServer.putVideo("driverCam", 0, 0);

  //  m_arm.setDefaultCommand(new InstantCommand(()-> m_arm.setShoulderSpeed(0.2), m_arm));//will change-MG
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
        m_chassis::getFusedPose,
        m_chassis::resetPose,
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(0.5, 0.0, 0.0),
        m_chassis::setSpeeds,
        m_eventMap,
        false, // BE AWARE OF AUTOMATIC MIRRORING, MAY CAUSE TRACKING PROBLEMS
        m_chassis);

    PathPlannerServer.startServer(5811);

    firstAutoCmd.setDefaultOption("Empty", AUTOS.EMPTY);
    firstAutoCmd.addOption("6Taxi", AUTOS.SIX_TAXI);
    firstAutoCmd.addOption("7Charge", AUTOS.SEVEN_CHARGE);
    firstAutoCmd.addOption("6ConeTaxi", AUTOS.SIX_CONE_TAXI);
    firstAutoCmd.addOption("7ConeCharge", AUTOS.SEVEN_CONE_CHARGE);

    SmartDashboard.putData("Auto Selector", firstAutoCmd);
    // #endregion
    
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    ArrayList<PathPoint> pathList = new ArrayList<PathPoint> ();
    pathList.add(new PathPoint(new Translation2d(0, 0), new Rotation2d(), new Rotation2d(), 0));
    pathList.add(new PathPoint(new Translation2d(1.5, 0), new Rotation2d(), new Rotation2d(), 0));

    new Trigger(m_driveController::getStartButton).onTrue(new InstantCommand(m_chassis::zeroGyro));

    new Trigger(m_driveController::getYButton).whileTrue(new AutoCenter(m_limeLight, new Pose2d(1.5, 0, new Rotation2d(0)), m_chassis));
    new Trigger(m_driveController::getXButton).whileTrue(new AutoCenter(m_limeLight, new Pose2d(1.5, -0.8, new Rotation2d(0)), m_chassis));
    new Trigger(m_driveController::getBButton).whileTrue(new AutoCenter(m_limeLight, new Pose2d(1.5, 0.8, new Rotation2d(0)), m_chassis));

    new Trigger(m_driveController::getAButton).whileTrue(new Park(m_chassis));

    new Trigger(m_driveController::getLeftBumper).whileTrue(new AutoChargeStation(m_chassis, 1, -0.7).andThen(new Park(m_chassis)));
    new Trigger(m_driveController::getRightBumper)
        .onTrue(new InstantCommand(() -> m_chassis.setCenter(new Translation2d(0.5, 0))));
    new Trigger(m_driveController::getRightBumper)
        .onFalse(new InstantCommand(() -> m_chassis.setCenter(new Translation2d(0, 0))));

    //Intake Buttons for V1 
    //new Trigger(m_manipController::getAButton).whileTrue(new RunTopNeos(m_IntakeV1_Lentz, -0.6));
    //new Trigger(m_manipController::getBButton).whileTrue(new RunBottomNeos(m_IntakeV1_Lentz, -0.3));
    //new Trigger(m_manipController::getXButton).whileTrue(new RunTopNeos(m_IntakeV1_Lentz, 0.6));
    //new Trigger(m_manipController::getYButton).whileTrue(new RunBottomNeos(m_IntakeV1_Lentz, 0.3));

    //mid cone score
   // new Trigger(m_manipController::getAButton).whileTrue(new SetArm(m_arm, 213.211, 123.627));
    //mid cube score
   // new Trigger(m_manipController::getBButton).whileTrue(new SetArm(m_arm, 113.845, 82.197));
    //floor pick up 
    //new Trigger(m_manipController::getYButton).whileTrue(new SetArm(m_arm, 120.974, 36.974 ));
    //stow 
   // new Trigger(m_manipController::getXButton).whileTrue(new SetArm(m_arm, 30.92, 37.581));

    //Button Map for Wasp Controls 
    //TOP LEFT TRIGGER --> ARM MID GRID PRESET
    new Trigger(m_manipController::getLeftBumper).whileTrue(new SetArm(m_arm, Constants.ELBOW_MID, Constants.SHOULDER_MID)).onFalse((new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW)).alongWith(new RunIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), Constants.HOLDSPEED)));
    //LOWER LEFT TRIGGER --> ARM LOW GRID

    // THIS HAS BEEN REPLACED WITH WHAT IS BELOW, BUT NEEDS TO BE REVIEWED
    // BooleanSupplier leftSupplier = new BooleanSupplier() {
    //   public boolean getAsBoolean() {
    //     return m_manipController.getLeftTriggerAxis() > .5;
    //   }};
    //   new Trigger(leftSupplier).whileTrue(new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR));
    // BooleanSupplier rightSupplier = new BooleanSupplier() {
    //   @Override
    //   public boolean getAsBoolean(){
    //     return m_manipController.getRightTriggerAxis() > 0.5;
    //   }
    // };
    // new Trigger(rightSupplier).whileTrue(new RunIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), -0.1));


    // AXIS TYPE NEEDS TO BE REVIEWED. NULL MIGHT ALSO NOT WORK
    new Trigger(m_manipController.axisGreaterThan(0, 0.5, null)).whileTrue(new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR));
    new Trigger(m_manipController.axisGreaterThan(1, 0.5, null)).whileTrue(new RunIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), -0.1));


    //ALL INTAKE BUTTONS WILL RETURN TO STOW POSITION AFTER COMPLETING INTAKE. iT IS THE LAST COMMAND IN SEQUENCE AFTER THE onFalse. 
    //Y BUTTON --> Shelf intake CONE
    new Trigger(m_manipController::getYButton).whileTrue((new SetArm(m_arm, Constants.ELBOW_SHELF, Constants.SHOULDER_SHELF)).alongWith(new RunIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0.35))).onFalse((new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW)).alongWith(new RunIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), 0)));
    //X BUTTON --> Floor Cube intake 
    new Trigger(m_manipController::getXButton).whileTrue((new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR)).alongWith(new RunIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, 0.3))).onFalse((new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW)).alongWith(new RunIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), Constants.HOLDSPEED)));
    //A BUTTON --> Floor Cone Intake
    new Trigger(m_manipController::getAButton).whileTrue((new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR)).alongWith(new RunIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0.35))).onFalse((new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW)).alongWith(new RunIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), 0)));
    //B BUTTON --> shelf Cube intake
    new Trigger(m_manipController::getBButton).whileTrue((new SetArm(m_arm, Constants.ELBOW_SHELF, Constants.SHOULDER_SHELF)).alongWith(new RunIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, 0.3))).onFalse((new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW)).alongWith(new RunIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), Constants.HOLDSPEED)));

    new Trigger(m_manipController::getRightBumper).toggleOnTrue(new RunIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, Constants.HOLDSPEED));

    //End of Intake buttons for V1

    // Intake buttons for Dave's intake (X = intake)

   //new Trigger(m_manipController::getXButton).whileTrue(new SetIntake(m_Dave_Intake, 0.6, DoubleSolenoid.Value.kForward)); 
   //new Trigger(m_manipController::getYButton).whileTrue(new SetIntake(m_Dave_Intake, 0.6, DoubleSolenoid.Value.kReverse)); 
   //new Trigger(m_manipController::getAButton).whileTrue(new SetIntake(m_Dave_Intake, -1 , DoubleSolenoid.Value.kReverse));
  }

  public Command getAutonomousCommand() {
    PathPlannerTrajectory test3 = PathFunctions.createTrajectory("Test3");
    PathPlannerTrajectory oneMeterStraight = PathFunctions.createTrajectory("1MeterStraight");
    PathPlannerTrajectory spiral = PathFunctions.createTrajectory("Spiral");  
    PathPlannerTrajectory eightEcho = PathFunctions.createTrajectory("8Echo");
    PathPlannerTrajectory echoEight = PathFunctions.createTrajectory("Echo8");
    PathPlannerTrajectory eightCharge = PathFunctions.createTrajectory("8Charge");
    PathPlannerTrajectory sixTaxi = PathFunctions.createTrajectory("6Taxi");
    PathPlannerTrajectory sevenCharge = PathFunctions.createTrajectory("7Charge");

    CommandBase autoCommand = null;

    switch (firstAutoCmd.getSelected()) {
      case EMPTY:
        autoCommand = new InstantCommand();
      break;

      case SIX_TAXI:
        autoCommand = new SequentialCommandGroup(
          new InstantCommand(() -> m_chassis.resetPose(sixTaxi.getInitialHolonomicPose())),
          autoBuilder.followPathWithEvents(sixTaxi)
          );
      break;

      case SEVEN_CHARGE:
        autoCommand = new SequentialCommandGroup(
          new AutoChargeStation(m_chassis, -1, 0.7),
          new Park(m_chassis)
        );
      break;

      case SIX_CONE_TAXI:
        autoCommand = new SequentialCommandGroup(
          new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR),
          new SetIntake(m_Dave_Intake, -0.25, DoubleSolenoid.Value.kForward),
          new WaitCommand(1),
          new SetIntake(m_Dave_Intake, Constants.HOLDSPEED, DoubleSolenoid.Value.kForward),
          new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW),
          new InstantCommand(() -> m_chassis.resetPose(sixTaxi.getInitialHolonomicPose())),
          autoBuilder.followPathWithEvents(sixTaxi)
        );
      break;

        case SEVEN_CONE_CHARGE:
        autoCommand = new SequentialCommandGroup(
          new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR),
          new SetIntake(m_Dave_Intake, -0.25, DoubleSolenoid.Value.kForward),
          new WaitCommand(1),
          new SetIntake(m_Dave_Intake, Constants.HOLDSPEED, DoubleSolenoid.Value.kForward),
          new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW),
          new InstantCommand(() -> m_chassis.resetPose(sixTaxi.getInitialHolonomicPose())),
          autoBuilder.followPathWithEvents(sixTaxi)
        );
      break;
    }
    return autoCommand;
  }
  /**
   * Applies a deadband to the given joystick axis value
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