// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Time;

import org.opencv.highgui.HighGui;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private SparkMaxAbsoluteEncoder shoulderEncoder;
  private SparkMaxAbsoluteEncoder elbowEncoder;
  private CANSparkMax shoulderNeo;
  private CANSparkMax elbowNeo;
  public PIDController elbowPIDcontroller;
  public PIDController shoulderPIDcontroller;
  public double lastTargetChangeTimestamp;
  public double elbowTarget;
  public double shoulderTarget; 
  private double lastElbowEncPos;
  private double lastShoulderEncPos;
  private double lastReadTime;
  private double elbowVel;
  private double shoulderVel;
  public DigitalInput shoulderLimitSwitch;

  /** Creates a new Arm. */
  public Arm() {
    shoulderNeo = new CANSparkMax(Constants.SHOULDER_NEO, MotorType.kBrushless);
    elbowNeo = new CANSparkMax(Constants.ELBOW_NEO, MotorType.kBrushless);

    shoulderEncoder = shoulderNeo.getAbsoluteEncoder(Type.kDutyCycle);
    elbowEncoder = elbowNeo.getAbsoluteEncoder(Type.kDutyCycle);
    elbowPIDcontroller = new PIDController(0.015, 0, 0);
    shoulderPIDcontroller = new PIDController(0.023, 0, 0);

    shoulderPIDcontroller.disableContinuousInput();
    shoulderPIDcontroller.setTolerance(2);
    elbowPIDcontroller.disableContinuousInput();
    elbowPIDcontroller.setTolerance(2);
    shoulderLimitSwitch = new DigitalInput(9);

    lastElbowEncPos = elbowEncoder.getPosition();
    lastShoulderEncPos = shoulderEncoder.getPosition();
    elbowVel = 0;
    shoulderVel = 0;
  }

  public void initialize() {
    elbowTarget = Constants.ELBOW_STOW;
    shoulderTarget = Constants.SHOULDER_STOW;
  }

  /**
   * sets the speed of the shoulder neo
   * @param motorPercentage value to set motor to, accepts -1 to 1
   */
public void setShoulderSpeed(double motorPercentage){
  shoulderNeo.set(motorPercentage);
}

/**
 * sets the speed of the elbow neo
 * @param motorPercentage value to set motor to, accepts -1 to 1
 */
public void setElbowSpeed(double motorPercentage){
  elbowNeo.set(motorPercentage);
}

/**
 * method to get absolute position of shoulder
 * @return double containing absolute position of shoulder
 */
public double getShoulderAbsolutePosition(){
  return (shoulderEncoder.getPosition() * 360) - Constants.SHOULDER_ENCODER_OFFSET;
}

/**
 * method to get absolute position of elbow
 * @return double containing absolute position of elbow
 */
public double getElbowAbsolutePosition(){
  return (elbowEncoder.getPosition() * 360) - Constants.ELBOW_ENCODER_OFFSET;
}

/**
 * method to get absolute position of shoulder
 * @return double containing absolute position of shoulder
 */
public double getShoulderVel(){
  return (shoulderEncoder.getPosition() * 360) - Constants.SHOULDER_ENCODER_OFFSET;
}

/**
 * method to get absolute position of elbow
 * @return double containing absolute position of elbow
 */
public double getElbowVel(){
  return (elbowEncoder.getPosition() * 360) - Constants.ELBOW_ENCODER_OFFSET;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shoulderEncoder", getShoulderAbsolutePosition());
    SmartDashboard.putNumber("elbowEncoder", getElbowAbsolutePosition());
    SmartDashboard.putNumber("targetShoulder", shoulderTarget);
    SmartDashboard.putNumber("targetElbow", elbowTarget);
    SmartDashboard.putNumber("shoulderError", shoulderPIDcontroller.getVelocityError());
    SmartDashboard.putNumber("elbowError", elbowPIDcontroller.getVelocityError());

    elbowVel = (elbowEncoder.getPosition() - lastElbowEncPos) / ((Timer.getFPGATimestamp() - lastReadTime));
    shoulderVel = (shoulderEncoder.getPosition() - lastShoulderEncPos) / ((Timer.getFPGATimestamp() - lastReadTime));
    lastElbowEncPos = elbowEncoder.getPosition();
    lastShoulderEncPos = shoulderEncoder.getPosition();
    lastReadTime = Timer.getFPGATimestamp();
  }

  public void elbowGoToPosition(double target){
    double elbowCurrentPosition = getElbowAbsolutePosition();
    if(elbowCurrentPosition > Constants.ELBOW_MIN && elbowCurrentPosition < Constants.ELBOW_MAX) {
      if(elbowCurrentPosition > (target + Constants.ELBOW_BUFFER)){
        setElbowSpeed(-1.0);
      }else if(elbowCurrentPosition < (target + Constants.ELBOW_BUFFER)){
        setElbowSpeed(1.0);
      }else{
        setElbowSpeed(0);
      }
    }
  }

  public boolean isLimitShoulder(){
    SmartDashboard.putBoolean("Limit switch", shoulderLimitSwitch.get());
    // SmartDashboard.putData("Input Limit", shoulderLimitSwitch.get());
    // return shoulderLimitSwitch.get();
    return false; // TEMPORARY, UNTIL LIMIT SWITCH SIGNAL WIRED
  }

  public void shoulderGoToPosition(double target){
    double shoulderCurrentPosition = getShoulderAbsolutePosition();
    if(shoulderCurrentPosition > target + Constants.SHOULDER_BUFFER){
      setElbowSpeed(-0.5);
    }else if(shoulderCurrentPosition < target + Constants.SHOULDER_BUFFER){
      setElbowSpeed(0.5);
    }else{
      setElbowSpeed(0);
    }
  }
}