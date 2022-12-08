package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

    //  MOTOR IDs
    //format: drive(Side L left or R right, Side U up or D down, Type D drive or H heading)
    public static final int driveLUD = 1;
    public static final int driveLUH = 2;

    public static final int driveRUD = 3;
    public static final int driveRUH = 4;
    
    public static final int driveLDD = 5;
    public static final int driveLDH = 6;

    public static final int driveRDD = 7;
    public static final int driveRDH = 8;

    //  SENSORS
    public static final int encLU = 9;
    public static final int encRU = 10;
    public static final int encLD = 11;
    public static final int encRD = 12;

    public static final int pigeonIMU = 0;
    public static final String photonCam = "photonvision";
    
    //  CONTROLLERS
    public static final int driverController = 0;

    //#region KINEMATICS
    //keep in mind that the locations for the modules must be relative to the center of the robot. Positive x values represent moving toward the front of the robot whereas positive y values represent moving toward the left of the robot.
    //order for swerve modules is always left upper, right upper, left lower, right lower
    public static final Translation2d wheelLU = new Translation2d(0.5588, 0.5588);
    public static final Translation2d wheelRU = new Translation2d(0.5588, -0.5588);
    public static final Translation2d wheelLD = new Translation2d(-0.5588, 0.5588);
    public static final Translation2d wheelRD = new Translation2d(-0.5588, -0.5588);

    // public static final double offsetLU = Math.toRadians(0) * -1; //put real offsets from smartdashboard
    // public static final double offsetRU = Math.toRadians(23) * -1;
    // public static final double offsetLD = Math.toRadians(106) * -1;
    // public static final double offsetRD = Math.toRadians(77) * -1; // maybe substract 180
    public static final double offsetLU = Math.toRadians(270) * -1; //put real offsets from smartdashboard
    public static final double offsetRU = Math.toRadians(293) * -1;
    public static final double offsetLD = Math.toRadians(16) * -1;
    public static final double offsetRD = Math.toRadians(343) * -1; // maybe substract 180

                                        //6380 is falcon FX max rpm, / 60 (to get revolutions per second), * gear ratio (to wheel rps), * wheel circumference
    public static final double maxSpeed = 6380.0 / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI; //meters per second
    public static final double maxVoltage = 12.0;
    public static final Mk4SwerveModuleHelper.GearRatio swerveGearRatio = Mk4SwerveModuleHelper.GearRatio.L2;
    //#endregion
    
     //#region PATH FOLLOWING
     public static final double maxVel = 1; //in meters per second
     public static final double maxAcc = 1; //in meters per second squared
     public static final double maxRotVel = 6.28; //6.28 is 1 rotation (it is 2xPI but I don't know why exactly)
     public static final double maxRotAcc = 3.14; //3.14 is half a rotation
     public static final double xErrVel = 1; // I don't know what "additional meter per second in the x direction for every meter of error in the x direction" means in the docs
     public static final double yErrVel = 1;
 
     public static final double kI = 0;
     public static final double kD = 0;
     //#endregion
}