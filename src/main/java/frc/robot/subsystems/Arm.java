// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.highgui.HighGui;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
  private double target;
  public double elbowTarget;
  public double shoulderTarget; 
  public DigitalInput shoulderLimitSwitch;

  /** Creates a new Arm. */
  public Arm() {
    shoulderNeo = new CANSparkMax(Constants.SHOULDER_NEO, MotorType.kBrushless);
    elbowNeo = new CANSparkMax(Constants.ELBOW_NEO, MotorType.kBrushless);
    shoulderEncoder = shoulderNeo.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    elbowEncoder = elbowNeo.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    elbowPIDcontroller = new PIDController(0.03, 0, 0);
    shoulderPIDcontroller = new PIDController(0.05, 0, 0);
    target = 0;

    shoulderPIDcontroller.disableContinuousInput();
    shoulderPIDcontroller.setTolerance(2);
    elbowPIDcontroller.disableContinuousInput();
    elbowPIDcontroller.setTolerance(2);
    shoulderLimitSwitch = new DigitalInput(9);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Encoder", getShoulderAbsolutePosition());
    SmartDashboard.putNumber("Elbow Encoder", getElbowAbsolutePosition());
    SmartDashboard.putNumber("targetShoulder", shoulderTarget);
    SmartDashboard.putNumber("targetElbow", elbowTarget);
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
      setShoulderSpeed(0.5);
    }else if(shoulderCurrentPosition < target + Constants.SHOULDER_BUFFER){
      setShoulderSpeed(-0.5);
    }else{
      setShoulderSpeed(0);
    }
  }
}