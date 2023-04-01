package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.classes.COTSFalconSwerveConstants;
import frc.robot.classes.SwerveModuleConstants;

public final class Constants {

    public static final int ELBOW_NEO = 10;
    public static final int SHOULDER_NEO = 9;
    public static final int LOWER_MANIP_NEO = 15;//should be 11
    public static final int UPPER_MANIP_NEO = 16;//should be 12
    public static final int DAVE_NEO = 11;
    /** A class within Constants that contains most of the swerve constants */
    public static final class Swerve {

        public static final COTSFalconSwerveConstants CHOSEN_MODULE =
         COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        // MK4 drivetrain constants
        // public static final double TRACK_WIDTH = Units.inchesToMeters(22);
        // public static final double WHEEL_BASE = Units.inchesToMeters(22);
        // Ant Man drivetrain constants
        public static final double TRACK_WIDTH = Units.inchesToMeters(18.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(18.75);
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;
        public static final boolean CANCODER_INVERT = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = CHOSEN_MODULE.angleKP;
        public static final double ANGLE_KI = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_KD = CHOSEN_MODULE.angleKD;
        public static final double ANGLE_KF = CHOSEN_MODULE.angleKF;
        public static final double ANGLE_MAX_ERR = CHOSEN_MODULE.angleMaxErr;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.05; // TODO: This must be tuned to specific robot
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double DRIVE_KS = (0.32 / 12); // TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = (1.51 / 12);
        public static final double DRIVE_KA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(47.63); //Wasp
            //  public static final Rotation2d angleOffset = Rotation2d.fromDegrees(8.70); // ant man
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
           public static final Rotation2d angleOffset = Rotation2d.fromDegrees(16.43); //WASP
            //  public static final Rotation2d angleOffset = Rotation2d.fromDegrees(162.77); // ant man
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(277.82); //Wasp
            //  public static final Rotation2d angleOffset = Rotation2d.fromDegrees(178.50); // ant man

            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;

           public static final Rotation2d angleOffset = Rotation2d.fromDegrees(110.65); //Wasp
            //  public static final Rotation2d angleOffset = Rotation2d.fromDegrees(165.93); // ant man
             
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final int PIGEON_IMU = 0;

    // PNEUMATICS
    public static final int FORWARD_CHANNEL = 2;
    public static final int BACKWARD_CHANNEL = 3;

    // CONTROL

    public static final int DRIVE_CONTROLLER = 0;
    public static final int MANIP_CONTROLLER = 1;
    public static final double DPAD_VEL = 2; // max meters per second (with RT down)

    // #region KINEMATICS

    // keep in mind that the locations for the modules must be relative to the
    // center of the robot. Positive x values represent moving toward the front of
    // the robot whereas positive y values represent moving toward the left of the
    // robot.
    // order for swerve modules is always left upper, right upper, left lower, right
    // lower

    // 6380 is falcon FX max rpm, / 60 (to get revolutions per second), * gear ratio
    // (to wheel rps), * wheel circumference
    public static final double MAX_SPEED = 6380.0 / 60.0 * Swerve.DRIVE_GEAR_RATIO * Units.inchesToMeters(Swerve.WHEEL_CIRCUMFERENCE) * Math.PI;
    // #endregion

    // #region AUTONOMOUS
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(6, 2.5);
    public static final double PATH_ROT_MAX_VEL = 6.28; // 6.28 is 1 rotation (it is 2xPI but I don't know why exactly)
    public static final double PATH_ROT_MAX_ACC = 3.14; // 3.14 is half a rotation
    public static final double X_ERROR_VEL = 1; // I don't know what "additional meter per second in the x direction for
    public static final double Y_ERROR_VEL = 1; // every meter of error in the x direction" means in the docs

    // AutoChargeStation and TraverseChargeStation constants
    public static final double CHARGE_SPEED = 1;
    public static final double MAX_TIME = 10;
    public static final double EXTRA_TIME = 0.5;
    // wooden values
    // public static final double REVERSE_SPEED = 0.7;
    // public static final double REVERSE_TIME = 0.8;
    // public static final double THRESHOLD = 9;
    // competition values
    public static final double REVERSE_SPEED = 0.7;
    public static final double REVERSE_TIME = 0.2;
    public static final double THRESHOLD = 8;

    public static final double TRAJECTORY_KI = 0;
    public static final double TRAJECTORY_KD = 0;
    // #endregion
   
    public static int SHOULDER_ENCODER = 0;
    public static int ELBOW_ENCODER = 1;

    //Constants for Arm Presets and such
    public static double ELBOW_SHELF = 213;
    public static double SHOULDER_SHELF = 220;

    public static double ELBOW_FLOOR = 117;
    public static double SHOULDER_FLOOR = 325;

    public static double ELBOW_MID = 230;
    public static double SHOULDER_MID = 264.8;

    public static double ELBOW_MID_DIAG_TELEOP = 120;
    public static double SHOULDER_MID_DIAG_TELEOP = 268;
    public static double ELBOW_MID_DIAG_AUTO_CUBE = 75;
    public static double SHOULDER_MID_DIAG_AUTO_CUBE = 308;
    public static double ELBOW_MID_DIAG_AUTO_CONE = 138.7; // teleop but shoulder is slightly less
    public static double SHOULDER_MID_DIAG_AUTO_CONE = 286;

    public static double ELBOW_STOW = 24;
    public static double SHOULDER_STOW = 329;

    public static double ELBOW_HIGH_CUBE = 140.48;
    public static double SHOULDER_HIGH_CUBE = 265.88;

    public static double HOLD_SPEED = 0.1;

    // TODO - change to proper default positions

    //  ARM
    //All encoder constants are offsets from a HOME position
    public static final double SHOULDER_MAX = 7800; // all max and mins subject to change after testing
    public static final double SHOULDER_MIN = -200;
    public static final double ELBOW_MAX = 7800;
    public static final double ELBOW_MIN = -200;

    //Deadzone Buffer for encoders
    public static final double ELBOW_BUFFER = 10;
    public static final double SHOULDER_BUFFER = 10;

    // NEED TO BE SET
    public static final double ELBOW_ENCODER_OFFSET = -6;
    public static final double SHOULDER_ENCODER_OFFSET = 0;

    //Arm Targets--> Subject to change (Delete when changed)
    public static final double SHOULDER_LOW_TARGET = 200;
    public static final double ELBOW_LOW_TARGET = 100;
    public static final double SHOULDER_MID_TARGET = 200;
    public static final double ELBOW_MID_TARGET = 100;

}
