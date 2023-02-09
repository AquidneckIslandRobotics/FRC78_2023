// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.classes.ArmFunctions;
import frc.robot.classes.JointConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class Arm extends SubsystemBase {

  protected DutyCycleEncoder shoulderEncoder;
  protected DutyCycleEncoder elbowEncoder;
  protected CANSparkMax shoulderNeo;
  protected CANSparkMax elbowNeo;
  protected SparkMaxPIDController elbowPIDcontroller;
  protected SparkMaxPIDController shoulderPIDcontroller;
  protected double target;
  protected ArmFunctions armFunctions;
  protected JointConfig elbow, shoulder;
  protected DCMotor neoDC;

  /** Creates a new Arm. */
  public Arm() {
    shoulderNeo = new CANSparkMax(Constants.SHOULDER_NEO, MotorType.kBrushless);
    elbowNeo = new CANSparkMax(Constants.ELBOW_NEO, MotorType.kBrushless);
    shoulderEncoder = new DutyCycleEncoder(Constants.SHOULDER_ENCODER);
    elbowEncoder = new DutyCycleEncoder(Constants.ELBOW_ENCODER);
    elbowPIDcontroller = elbowNeo.getPIDController();
    shoulderPIDcontroller = shoulderNeo.getPIDController();
    target = 0;
    neoDC = new DCMotor(12, 2.6, 105, 1.8, (5676 / 60) * 6.28319, 1);
    shoulder = new JointConfig(target, target, target, target, target, target, target, null);
    armFunctions = new ArmFunctions(shoulder, elbow);
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
  return (shoulderEncoder.getAbsolutePosition() * 360) - Constants.SHOULDER_ENCODER_OFFSET;
}

/**
 * method to get absolute position of elbow
 * @return double containing absolute position of elbow
 */
public double getElbowAbsolutePosition(){
  return (elbowEncoder.getAbsolutePosition() * 360) - Constants.ELBOW_ENCODER_OFFSET;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Encoder", getShoulderAbsolutePosition());
    SmartDashboard.putNumber("Elbow Encoder", getElbowAbsolutePosition());
  }

  public void elbowGoToPosition(double target){
    double elbowCurrentPosition = getElbowAbsolutePosition();
    if(elbowCurrentPosition > Constants.ELBOW_MIN && elbowCurrentPosition < Constants.ELBOW_MAX) {
      if(elbowCurrentPosition > (target + Constants.ELBOW_BUFFER)){
        setElbowSpeed(-0.5);
      }else if(elbowCurrentPosition < (target + Constants.ELBOW_BUFFER)){
        setElbowSpeed(0.5);
      }else{
        setElbowSpeed(0);
      }
    }
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