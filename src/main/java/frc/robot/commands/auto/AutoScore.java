// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dave_Intake;

public class AutoScore extends SequentialCommandGroup {
  public static enum scoreLevel {LOW, MID, HIGH};
  // public static enum gamepiece {CUBE, CONE};

  /** Creates a new AutoScore. */
  public AutoScore(Arm arm, Dave_Intake intake, scoreLevel level/**, gamepiece piece */) {
    switch (level) {
      case LOW:
        addCommands(new SetArm(arm, Constants.ELBOW_FLOOR, Constants.SHOULDER_FLOOR));
      break;
      case MID:
        addCommands(new SetArm(arm, Constants.ELBOW_MID, Constants.SHOULDER_MID));
      case HIGH:
        addCommands(new SetArm(arm, Constants.ELBOW_HIGH_CUBE, Constants.SHOULDER_HIGH_CUBE));
      break;
    }
    // switch (piece) {
    //   case CUBE:
    //   addCommands(new SetIntake(intake, DoubleSolenoid.Value.kReverse, -0.1));
    //   break;
    //   case CONE:
    //   addCommands(new SetIntake(intake, DoubleSolenoid.Value.kReverse, -0.1));
    //   break;
    // }
    addCommands(
      new SetIntake(intake, DoubleSolenoid.Value.kReverse, -0.1),
      new WaitCommand(0.5),
      new SetIntake(intake, DoubleSolenoid.Value.kForward, 0),
      new SetArm(arm, Constants.ELBOW_STOW, Constants.SHOULDER_STOW)
    );
  }
}
