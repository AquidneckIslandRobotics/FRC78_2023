// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore extends SequentialCommandGroup {
  Arm m_Arm;
  Dave_Intake m_Dave_Intake;
  /** Creates a new AutoScoreConeExit. */
  public AutoScore(SwerveChassis chassis, Arm m_Arm, double elbow, double shoulder, 
                Dave_Intake m_Dave_Intake, double speed,
                double elbow2, double shoulder2) {
    List<PathPoint> pathLists;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetArmAuto(m_Arm, elbow, shoulder), new SetIntake(m_Dave_Intake, speed , DoubleSolenoid.Value.kForward), new AutoSwerve(chassis), new SetArmAuto(m_Arm, elbow2, shoulder2));
  }
}
