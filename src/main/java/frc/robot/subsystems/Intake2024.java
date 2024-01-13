// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake2024 extends SubsystemBase {
  private CANSparkMax NeoTopRoller;
  private CANSparkMax NeoBottomRoller;

  /** Creates a new Intake2024. */
  public Intake2024() {
    NeoTopRoller = new CANSparkMax(13, MotorType.kBrushless);
    NeoBottomRoller  = new CANSparkMax(14, MotorType.kBrushless);
  }
  public void setTopNeoSpeed(double topSpeed){
    NeoTopRoller.set(topSpeed);
  }

  public void setBottomNeoSpeed(double bottomSpeed){
    NeoBottomRoller.set(bottomSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
