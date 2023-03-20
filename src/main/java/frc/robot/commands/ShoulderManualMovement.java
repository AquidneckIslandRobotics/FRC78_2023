// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ShoulderManualMovement extends CommandBase {
  //protected final XboxController armController;
  protected final Arm m_Arm;
  private double joystickValue;
  /** Creates a new ShoulderManualMovement. */
  public ShoulderManualMovement(Arm m_Arm, double joystickValue) {
   // armController = m_armController;
    this.m_Arm = m_Arm;
    this.joystickValue = joystickValue;
    addRequirements(m_Arm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setShoulderSpeed(joystickValue * 0.5);

   // m_Arm.setElbowSpeed(armController.getLeftY() * 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
