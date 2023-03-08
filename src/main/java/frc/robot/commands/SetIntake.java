// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DaveIntake;

public class SetIntake extends CommandBase {
  private DaveIntake intake;
  private intakeMode intakeMode;

  public enum intakeMode {CUBE_HOLD, CONE_HOLD, CUBE_INTAKE, CONE_INTAKE, OUTTAKE, CUBE_HIGH_OUTTAKE};

  public SetIntake(DaveIntake intake, intakeMode intakeMode) {
    this.intake = intake;
    this.intakeMode = intakeMode;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    switch (intakeMode) {
      case CUBE_HOLD: {
        intake.setSpeed(Constants.HOLD_SPEED);
        intake.setSolenoid(DoubleSolenoid.Value.kReverse);
        break;
      }
      case CONE_HOLD: {
        intake.setSpeed(0);
        intake.setSolenoid(DoubleSolenoid.Value.kForward);
        break;
      }
      case CUBE_INTAKE: {
        intake.setSpeed(0.3);
        intake.setSolenoid(DoubleSolenoid.Value.kReverse);
        break;
      }
      case CONE_INTAKE: {
        intake.setSpeed(0.35);
        intake.setSolenoid(DoubleSolenoid.Value.kForward);
        break;
      }
      case OUTTAKE: {
        if (intake.getSolenoid() == DoubleSolenoid.Value.kReverse) {
          intake.setSpeed(-0.1);
        } else {
          intake.setSpeed(0);
          intake.setSolenoid(DoubleSolenoid.Value.kReverse);
        }
        break;
      }
      case CUBE_HIGH_OUTTAKE: {
        intake.setSpeed(-0.5);
        intake.setSolenoid(DoubleSolenoid.Value.kReverse);
        break;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}