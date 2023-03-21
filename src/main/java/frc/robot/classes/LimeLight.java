// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class LimeLight {
    NetworkTable table;
    DoubleArraySubscriber camPose;
    DoubleArraySubscriber blueFieldPose;
    DoubleArraySubscriber redFieldPose;

    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tid;

    public LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        camPose = table.getDoubleArrayTopic("campose").subscribe(new double[] {});
        blueFieldPose = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        redFieldPose = table.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});
    
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tid = table.getEntry("tid");
    }

    //TODO might have null pointer errors if run too early
    public Pose2d getBotPose() {
        double[] array;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            array = blueFieldPose.get();
        } else {
            array = redFieldPose.get();
        }

        SmartDashboard.putNumber("PoseX", array[0]);
        SmartDashboard.putNumber("PoseY", array[1]);
        SmartDashboard.putNumber("PoseZ", array[2]);
        SmartDashboard.putNumber("PoseXRot", array[3]);
        SmartDashboard.putNumber("PoseYRot", array[4]);
        SmartDashboard.putNumber("PoseZRot", array[5]);

        return new Pose2d(array[0], array[1], Rotation2d.fromDegrees(array[5]));
    }

    public boolean hasApriltag() {
        return tid.getDouble(-1) != -1; 
    }

    public long getBotPoseTimestamp() {
        long timestamp;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            timestamp = blueFieldPose.getAtomic().timestamp; // might have to be switched to server timestamp
        } else {
            timestamp = redFieldPose.getAtomic().timestamp; // might have to be switched to server timestamp
        }
        SmartDashboard.putNumber("lastBotPoseTimestamp", timestamp);
        return timestamp;
    }

    public Pose2d getCamPose() {
        double[] array = camPose.get();
        return new Pose2d(array[2], array[0], Rotation2d.fromDegrees(array[4]));
    }
}
