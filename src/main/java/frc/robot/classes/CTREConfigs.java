package frc.robot.classes;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.ANGLE_ENABLE_CURRENT_LIMIT, 
            Constants.Swerve.ANGLE_CONTINOUS_CURRENT_LIMIT, 
            Constants.Swerve.ANGLE_PEAK_CURRENT_LIMIT, 
            Constants.Swerve.ANGLE_PEAK_CURRENT_DURATION);

        swerveAngleFXConfig.slot0.kP = Constants.Swerve.ANGLE_KP;
        swerveAngleFXConfig.slot0.kI = Constants.Swerve.ANGLE_KI;
        swerveAngleFXConfig.slot0.kD = Constants.Swerve.ANGLE_KD;
        swerveAngleFXConfig.slot0.kF = Constants.Swerve.ANGLE_KF;
        // swerveAngleFXConfig.slot0.allowableClosedloopError = Constants.Swerve.ANGLE_MAX_ERR;
        SmartDashboard.putNumber("alloable error", swerveAngleFXConfig.slot0.allowableClosedloopError);
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.Swerve.DRIVE_CONTINOUS_CURRENT_LIMIT, 
            Constants.Swerve.DRIVE_PEAK_CURRENT_LIMIT, 
            Constants.Swerve.DRIVE_PEAK_CURRENT_DURATION);

        swerveDriveFXConfig.slot0.kP = Constants.Swerve.DRIVE_KP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.DRIVE_KI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.DRIVE_KD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.DRIVE_KF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.CLOSED_LOOP_RAMP;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.CANCODER_INVERT;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}