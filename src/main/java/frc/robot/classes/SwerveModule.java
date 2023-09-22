package frc.robot.classes;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * This is our custom class that represents an entire swerve module.
 * It contains functions for getting and setting modules, as well as a few other
 * things.
 */
public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private SparkMaxPIDController mAnglePidController;
    private SparkMaxPIDController mDrivePidController;
    //private CANCoder angleEncoder;
    private SparkMaxAbsoluteEncoder angleEncoder;
 

    public double CANcoderInitTime = 0.0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Motor Config */
        // mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "drivetrainCAN");
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

         /* Angle Encoder Config */
        // angleEncoder = new CANCoder(moduleConstants.cancoderID, "drivetrainCAN");
        angleEncoder = mAngleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        configAngleEncoder();

        /* Drive Motor Config */
        // mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "drivetrainCAN");
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        mAnglePidController = mAngleMotor.getPIDController();
        
        mDrivePidController = mDriveMotor.getPIDController();

        mAnglePidController.setP(Constants.Swerve.ANGLE_KP);
        mAnglePidController.setI(Constants.Swerve.ANGLE_KI);
        mAnglePidController.setD(Constants.Swerve.ANGLE_KD);
        mAnglePidController.setIZone(0);
        mAnglePidController.setFF(Constants.Swerve.ANGLE_KF);
        mAnglePidController.setOutputRange(-1, 1);

        mDrivePidController.setP(Constants.Swerve.DRIVE_KP);
        mDrivePidController.setI(Constants.Swerve.DRIVE_KI);
        mDrivePidController.setD(Constants.Swerve.DRIVE_KD);
        mDrivePidController.setIZone(0);
        mDrivePidController.setFF(Constants.Swerve.DRIVE_KF);
        mDrivePidController.setOutputRange(-1, 1);

    

        lastAngle = getState().angle;
    }

    /**
     * Sets the state of the module
     * 
     * @param desiredState
     * @param isOpenLoop
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideDeadband) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = Calculations.optimize(desiredState, getState().angle);
        setAngle(desiredState, overrideDeadband);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            mDriveMotor.set(percentOutput);
        } else {
            double velocity = Calculations.MPSToFalcon(desiredState.speedMetersPerSecond,//initial falcon velocity and behind the scene math. Takes in desired speed with wheel an GR
                    Constants.Swerve.WHEEL_CIRCUMFERENCE, Constants.Swerve.DRIVE_GEAR_RATIO);
            double setPoint = velocity;                         //Sets the variable "set point" to the velocity above, allowing NEO to access this velocity
            //Uses the pid controller through the neos to basically control the velocity as falcon
            mDrivePidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean overrideDeadband) {
        Rotation2d angle;
        if (!overrideDeadband) {
            angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01))
                    ? lastAngle
                    : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        } else {
            angle = desiredState.angle;
        }

        //mAngleMotor.set(Calculations.degreesToFalcon(angle.getDegrees(), Constants.Swerve.ANGLE_GEAR_RATIO));
        mAnglePidController.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
        lastAngle = angle;
    }

    //private Rotation2d getAngle() {
       // return Rotation2d.fromDegrees(Calculations.falconToDegrees(mAngleMotor.getSelectedSensorPosition(),
             //   Constants.Swerve.ANGLE_GEAR_RATIO));
    //}

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

   // public void resetToAbsolute() {
       // double absolutePosition = Calculations.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(),
         //       Constants.Swerve.ANGLE_GEAR_RATIO);
       // mAngleMotor.setPosition(absolutePosition);
   // } This is all commented out because we shouldn't need to reset, with the abosulte encoder we will just need to use relativity. --MG 8/2

    private void configAngleEncoder() {
        //angleEncoder.configFactoryDefault();
        //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {//This is all com,mented out beacause we ha ve configured it earlier in the code. see line 63. --MG 8/2
        //mAngleMotor.restoreFactoryDefaults()
        //mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
       // mAngleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
       // mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        //resetToAbsolute();
    }

    private void configDriveMotor() {//All commented for same reason see line 146
       // mDriveMotor.configFactoryDefault();
        //mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
       // mDriveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
       // mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        //mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Calculations.RPMToMPS(mDriveMotor.getEncoder().getVelocity(), Constants.Swerve.WHEEL_CIRCUMFERENCE,
                        Constants.Swerve.DRIVE_GEAR_RATIO),
                getCanCoder());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Calculations.RotationsToMeters(mDriveMotor.getEncoder().getPosition(),
                        Constants.Swerve.WHEEL_CIRCUMFERENCE, Constants.Swerve.DRIVE_GEAR_RATIO),
                getCanCoder());
    }

    public double getDriveCurrent() {
        return mDriveMotor.getBusVoltage() * mDriveMotor.getAppliedOutput();
    }

    public double getSteerCurrent() {
        return mAngleMotor.getBusVoltage() * mAngleMotor.getAppliedOutput();
    }

}