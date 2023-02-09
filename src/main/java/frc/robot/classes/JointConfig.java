// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.system.plant.DCMotor;


// public static class static JointConfig (
//   double mass,
//   double length,
//   double moi,
//   double cgRadius,
//   double minAngle,
//   double maxAngle,
//   double reduction,
//   DCMotor motor
// )
/** Add your docs here. */
public class JointConfig {
  protected double mass, length, moi, cgRadius, minAngle, maxAngle, reduction;
  protected DCMotor motor;

    public JointConfig(
      double mass,
      double length,
      double moi,
      double cgRadius,
      double minAngle,
      double maxAngle,
      double reduction,
      DCMotor motor)
      {
        this.mass = mass;
        this.length = length;
        this.moi = moi;
        this.cgRadius = cgRadius;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.reduction = reduction;
        this.motor = motor;
      }
}