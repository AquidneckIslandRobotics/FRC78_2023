// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dave_Intake extends SubsystemBase {
  public CANSparkMax leftNeo;
  public CANSparkMax rightNeo;
  /** Creates a new IntakeV1_Lentz. */
  public Dave_Intake() {
    leftNeo = new CANSparkMax(6, MotorType.kBrushless);
    rightNeo = new CANSparkMax(5, MotorType.kBrushless);
  }
 
  public void setSpeed(double speed) {
    leftNeo.set(speed);
    rightNeo.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


