package frc.robot.classes;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    public double CANcoderInitTime = 0.0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward (
        Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, "drivetrainCAN");
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "drivetrainCAN");
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "drivetrainCAN");
        configDriveMotor();

        lastAngle = getState().angle;
    }

    /**
     * Sets the state of the module
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
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Calculations.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.Swerve.WHEEL_CIRCUMFERENCE, Constants.Swerve.DRIVE_GEAR_RATIO);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean overrideDeadband) {
        Rotation2d angle;
        if(!overrideDeadband) {
            angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01))
            ? lastAngle
            : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        } else {
            angle = desiredState.angle;
        }
        

        mAngleMotor.set(ControlMode.Position,
                Calculations.degreesToFalcon(angle.getDegrees(), Constants.Swerve.ANGLE_GEAR_RATIO));
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Calculations.falconToDegrees(mAngleMotor.getSelectedSensorPosition(),
                Constants.Swerve.ANGLE_GEAR_RATIO));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = Calculations.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(),
                Constants.Swerve.ANGLE_GEAR_RATIO);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Calculations.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.WHEEL_CIRCUMFERENCE,
                        Constants.Swerve.DRIVE_GEAR_RATIO),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Calculations.falconToMeters(mDriveMotor.getSelectedSensorPosition(),
                        Constants.Swerve.WHEEL_CIRCUMFERENCE, Constants.Swerve.DRIVE_GEAR_RATIO),
                getAngle());
    }
}