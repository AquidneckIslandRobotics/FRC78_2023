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
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.classes.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class RobotContainer {

  public final SwerveChassis m_chassis;
  public final Arm m_arm;
  private final LimeLight m_limeLight;
  private final UsbCamera m_driverCam;
  public final MjpegServer m_mjpegServer;
  
  private final XboxController m_driveController;
  private final XboxController m_manipController;
  private final XboxController m_testController;
  //private final IntakeV1_Lentz m_IntakeV1_Lentz;
  private final Dave_Intake m_Dave_Intake;
  private final HashMap<String, Command> m_eventMap;
  private final SwerveAutoBuilder autoBuilder;

  static enum AUTOS {EMPTY, SIX_TAXI, SEVEN_CHARGE, SIX_CONE_TAXI, CONE_TAXI_CHARGE, CONE_PICKUP_CONE, CUBE_HIGH_CHARGE_TAXI, CONE_TAXI_EIGHT, CONE_PICKUP_CONE_EIGHT};
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

    m_testController = new XboxController(5);

    m_chassis.setDefaultCommand(new SwerveDrive(
        m_chassis,
        () -> -modifyAxis(m_driveController.getLeftY()),
        () -> -modifyAxis(m_driveController.getLeftX()),
        () -> -modifyAxis(m_driveController.getRightX()),
        () -> m_driveController.getPOV(),
        () -> modifyAxis(m_driveController.getLeftTriggerAxis()),
        () -> modifyAxis(m_driveController.getRightTriggerAxis())));

   m_arm.setDefaultCommand(new SetArmPID(m_arm));

   m_driverCam = CameraServer.startAutomaticCapture();
   m_driverCam.setResolution(256, 192);
  //  m_driverCam.setResolution(64, 43);
   m_mjpegServer = new MjpegServer("driverCamServer", 1181);
   m_mjpegServer.setSource(m_driverCam);
    //  CvSink cvSink = CameraServer.getVideo();
    //  CvSource outputStream = CameraServer.putVideo("driverCam", 0, 0);

  //  m_arm.setDefaultCommand(new InstantCommand(()-> m_arm.setShoulderSpeed(0.2), m_arm));//will change-MG
    // m_Dave_Intake.setDefaultCommand(new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0.1));

    Trigger buttonA = new JoystickButton(m_manipController, XboxController.Button.kX.value);
    buttonA.onTrue(new InstantCommand(() -> new SetArm(m_arm, Constants.SHOULDER_LOW_TARGET, Constants.ELBOW_LOW_TARGET)));
    buttonA.onFalse(new InstantCommand(() -> m_arm.setShoulderSpeed(0)));
    
    Trigger buttonB = new JoystickButton(m_manipController, XboxController.Button.kX.value);
    buttonB.onTrue(new InstantCommand(() -> new SetArm(m_arm, Constants.SHOULDER_MID_TARGET, Constants.ELBOW_MID_TARGET)));
    buttonB.onFalse(new InstantCommand(() -> m_arm.setShoulderSpeed(0)));
  

    // #region PATHPLANNER
    m_eventMap = new HashMap<>();
    m_eventMap.put("Waypoint1Reached", new PrintCommand("Waypoint 1 reached!"));
    // m_eventMap.put("armPickupCone", new ParallelCommandGroup(
    //   new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR),
    //   new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0.2))
    //   );
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
    firstAutoCmd.addOption("Taxi (6)", AUTOS.SIX_TAXI);
    firstAutoCmd.addOption("Charge (7)", AUTOS.SEVEN_CHARGE);
    firstAutoCmd.addOption("Cone Taxi (6)", AUTOS.SIX_CONE_TAXI);
    firstAutoCmd.addOption("ConeTaxiCharge (7)", AUTOS.CONE_TAXI_CHARGE);
    firstAutoCmd.addOption("ConePickupCone (6)", AUTOS.CONE_PICKUP_CONE);
    firstAutoCmd.addOption("Cube High Taxi Charge (7)", AUTOS.CUBE_HIGH_CHARGE_TAXI);
    firstAutoCmd.addOption("Cone Taxi (8)", AUTOS.CONE_TAXI_EIGHT);
    firstAutoCmd.addOption("Cone Pickup Cone (8)", AUTOS.CONE_PICKUP_CONE_EIGHT);

    SmartDashboard.putData("Auto Selector", firstAutoCmd);
    // #endregion
    
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    ArrayList<PathPoint> pathList = new ArrayList<PathPoint> ();
    pathList.add(new PathPoint(new Translation2d(0, 0), new Rotation2d(), new Rotation2d(), 0));
    pathList.add(new PathPoint(new Translation2d(1.5, 0), new Rotation2d(), new Rotation2d(), 0));

    new Trigger(m_driveController::getStartButton).onTrue(new InstantCommand(() -> m_chassis.resetPose(new Pose2d(m_chassis.getFusedPose().getX(), m_chassis.getFusedPose().getY(), new Rotation2d()))));
    new Trigger(m_driveController::getYButton).whileTrue(new AutoCenter(m_limeLight, new Pose2d(1.5, 0, new Rotation2d(0)), m_chassis));
    new Trigger(m_driveController::getXButton).whileTrue(new AutoCenter(m_limeLight, new Pose2d(1.5, -0.8, new Rotation2d(0)), m_chassis));
    new Trigger(m_driveController::getBButton).whileTrue(new AutoCenter(m_limeLight, new Pose2d(1.5, 0.8, new Rotation2d(0)), m_chassis));
    new Trigger(m_driveController::getRightBumper)
        .onTrue(new InstantCommand(() -> m_chassis.setCenter(new Translation2d(0.5, 0))));
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
    //new Trigger(m_manipController::getAButton).whileTrue(new RunTopNeos(m_IntakeV1_Lentz, -0.6));
    //new Trigger(m_manipController::getBButton).whileTrue(new RunBottomNeos(m_IntakeV1_Lentz, -0.3));
    //new Trigger(m_manipController::getXButton).whileTrue(new RunTopNeos(m_IntakeV1_Lentz, 0.6));
    //new Trigger(m_manipController::getYButton).whileTrue(new RunBottomNeos(m_IntakeV1_Lentz, 0.3));

    //subStation Pick up

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
    new Trigger(m_manipController::getLeftBumper).whileTrue(new SetArm(m_arm, Constants.ELBOW_MID, Constants.SHOULDER_MID)).onFalse((new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW)).alongWith(new SetIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), Constants.HOLD_SPEED)));
    //LOWER LEFT TRIGGER --> ARM LOW GRID
    BooleanSupplier leftSupplier = new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return m_manipController.getLeftTriggerAxis() > .5;
      }};
    BooleanSupplier rightSupplier = new BooleanSupplier() {
      @Override
      public boolean getAsBoolean(){
        return m_manipController.getRightTriggerAxis() > 0.5;
      }
    };
    new Trigger(leftSupplier).whileTrue(new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR));
    new Trigger(rightSupplier).whileTrue(new SetIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), -0.1));
    //ALL INTAKE BUTTONS WILL RETURN TO STOW POSITION AFTER COMPLETING INTAKE. iT IS THE LAST COMMAND IN SEQUENCE AFTER THE onFalse. 
    //Y BUTTON --> Shelf intake CONE
    new Trigger(m_manipController::getYButton).whileTrue((new SetArm(m_arm, Constants.ELBOW_SHELF, Constants.SHOULDER_SHELF)).alongWith(new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0.35))).onFalse((new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW)).alongWith(new SetIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), 0)));
    //X BUTTON --> Floor Cube intake 
    new Trigger(m_manipController::getXButton).whileTrue((new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR)).alongWith(new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, 0.3))).onFalse((new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW)).alongWith(new SetIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), Constants.HOLD_SPEED)));
    //A BUTTON --> Floor Cone Intake
    new Trigger(m_manipController::getAButton).whileTrue((new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR)).alongWith(new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0.35))).onFalse((new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW)).alongWith(new SetIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), 0)));
    //B BUTTON --> shelf Cube intake
    new Trigger(m_manipController::getBButton).whileTrue((new SetArm(m_arm, Constants.ELBOW_SHELF, Constants.SHOULDER_SHELF)).alongWith(new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, 0.3))).onFalse((new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW)).alongWith(new SetIntake(m_Dave_Intake, m_Dave_Intake.getSolenoid(), Constants.HOLD_SPEED)));
    
    new Trigger(m_manipController::getRightBumper).toggleOnTrue(new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, Constants.HOLD_SPEED));

    //End of Intake buttons for V1
    // Intake buttons for Dave's intake (X = intake)

   //new Trigger(m_manipController::getXButton).whileTrue(new SetIntake(m_Dave_Intake, 0.6, DoubleSolenoid.Value.kForward)); 
   //new Trigger(m_manipController::getYButton).whileTrue(new SetIntake(m_Dave_Intake, 0.6, DoubleSolenoid.Value.kReverse)); 
   //new Trigger(m_manipController::getAButton).whileTrue(new SetIntake(m_Dave_Intake, -1 , DoubleSolenoid.Value.kReverse));
    new Trigger(m_driveController::getAButton).whileTrue(new Park(m_chassis));
    new Trigger(m_driveController::getLeftBumper).whileTrue(new AutoChargeStation(m_chassis, 1).andThen(new Park(m_chassis)));
    new Trigger(() -> m_driveController.getRawButton(3)).whileTrue( //BUTTON NEEDS TO BE SET TO THE PROPER ID
        autoBuilder.followPath(PathPlanner.generatePath(
            new PathConstraints(1, 1), pathList)));

    //TestController Buttons 
    new Trigger(m_testController::getAButton).whileTrue((new SetArm(m_arm, Constants.ELBOW_HIGH_CUBE, Constants.SHOULDER_HIGH_CUBE)).alongWith(new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, Constants.HOLD_SPEED))).onFalse((new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_HIGH_CUBE)));
    new Trigger(m_testController::getBButton).whileTrue((new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, -0.5))).onFalse(new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, Constants.HOLD_SPEED));

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
    PathPlannerTrajectory sixPickup = PathFunctions.createTrajectory("6Pickup");
    PathPlannerTrajectory pickupSix = PathFunctions.createTrajectory("Pickup6");
    PathPlannerTrajectory eightHotel = PathFunctions.createTrajectory("8 Hotel");
    PathPlannerTrajectory hotelEight = PathFunctions.createTrajectory("Hotel 8");

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
          new InstantCommand(() -> m_chassis.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
          new AutoChargeStation(m_chassis, -1)
        );
      break;
      case SIX_CONE_TAXI:
        SmartDashboard.putString("output 6 cone", "I AM HERE");
        autoCommand = new SequentialCommandGroup(
          
          new InstantCommand(() -> m_chassis.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0),
          new SetArm(m_arm, Constants.ELBOW_MID, Constants.SHOULDER_MID),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, -0.1),
          new WaitCommand(0.5),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0),
          new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW),
          new InstantCommand(() -> m_chassis.resetPose(sixTaxi.getInitialHolonomicPose())),
          autoBuilder.followPathWithEvents(sixTaxi)
          //MAY RUN INTO BARRIAR CHECK AT WPI--- maddie 2-22-23
        );
      break;
      case CONE_TAXI_CHARGE:
        autoCommand = new SequentialCommandGroup(
          new InstantCommand(() -> m_chassis.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0),
          new SetArm(m_arm, Constants.ELBOW_MID, Constants.SHOULDER_MID),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, -0.1),
          new WaitCommand(0.5),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0),
          new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW),
          new TraverseChargeStation(m_chassis, -Constants.CHARGE_SPEED),
          new WaitCommand(1),
          new AutoChargeStation(m_chassis, Constants.CHARGE_SPEED),
          new Park(m_chassis)
        );
      break;
      case CONE_PICKUP_CONE:
        autoCommand = new SequentialCommandGroup(
          new InstantCommand(() -> m_chassis.resetPose(sixPickup.getInitialHolonomicPose())),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0),
          new SetArm(m_arm, Constants.ELBOW_MID, Constants.SHOULDER_MID),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, -0.1),
          new WaitCommand(0.5),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0.2),
          new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW),
          new ParallelCommandGroup(
            autoBuilder.followPathWithEvents(sixPickup),
            new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR)
          ),
          new ParallelCommandGroup(
            autoBuilder.followPathWithEvents(pickupSix),
            new SetArm(m_arm, Constants.ELBOW_MID, Constants.SHOULDER_MID)
          ),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, -0.1)
        );
      break;
      case CUBE_HIGH_CHARGE_TAXI:
      autoCommand = new SequentialCommandGroup(
        new InstantCommand(() -> m_chassis.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
        new ParallelCommandGroup(
          new SetArm(m_arm, Constants.ELBOW_HIGH_CUBE, Constants.SHOULDER_HIGH_CUBE),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, Constants.HOLD_SPEED)
        ),
        new WaitCommand(0.25),
        new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, -0.5),
        new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW),
        new TraverseChargeStation(m_chassis, -Constants.CHARGE_SPEED),
        new WaitCommand(1),
        new AutoChargeStation(m_chassis, Constants.CHARGE_SPEED),
        new Park(m_chassis)
      );
      break;
      case CONE_TAXI_EIGHT:
      autoCommand = new SequentialCommandGroup(
        new InstantCommand(() -> m_chassis.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0),
          new SetArm(m_arm, Constants.ELBOW_MID, Constants.SHOULDER_MID),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, -0.1),
          new WaitCommand(0.5),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0),
          new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW),
          new InstantCommand(() -> m_chassis.resetPose(sixTaxi.getInitialHolonomicPose())),
          autoBuilder.followPathWithEvents(eightHotel)
      );
      break;
      case CONE_PICKUP_CONE_EIGHT:
      autoCommand = new SequentialCommandGroup(
        new InstantCommand(() -> m_chassis.resetPose(sixPickup.getInitialHolonomicPose())),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0),
          new SetArm(m_arm, Constants.ELBOW_MID, Constants.SHOULDER_MID),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, -0.1),
          new WaitCommand(0.5),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kForward, 0.2),
          new SetArm(m_arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW),
          new ParallelCommandGroup(
            autoBuilder.followPathWithEvents(eightHotel),
            new SetArm(m_arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR)
          ),
          new ParallelCommandGroup(
            autoBuilder.followPathWithEvents(hotelEight),
            new SetArm(m_arm, Constants.ELBOW_MID, Constants.SHOULDER_MID)
          ),
          new SetIntake(m_Dave_Intake, DoubleSolenoid.Value.kReverse, -0.1)
      );


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