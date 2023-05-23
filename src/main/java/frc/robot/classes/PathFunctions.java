package frc.robot.classes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

/** Add your docs here. */
public class PathFunctions {
    /**
     * Returns a Trajectory object, given a path to the trajectory file
     * 
     * @param path trajectory file name, without file extension
     * @return Trajectory object, transformed for the current alliance
     */
    public static PathPlannerTrajectory createTrajectory(String path) {
        return PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath(path, Constants.PATH_CONSTRAINTS), DriverStation.getAlliance());
    }

    /**
     * Resets the odometry of the robot to the provided trajectory intial pose
     * 
     * @param m_chassis
     * @param trajectory
     */
    public static CommandBase resetOdometry(SwerveChassis m_chassis, PathPlannerTrajectory trajectory) {
        return new InstantCommand(() -> m_chassis.resetPose(trajectory.getInitialHolonomicPose()));
    }
    public static CommandBase resetOdometryPos(SwerveChassis m_chassis, PathPlannerTrajectory trajectory) {
        return new InstantCommand(() -> m_chassis.resetPose(new Pose2d(trajectory.getInitialHolonomicPose().getX(), trajectory.getInitialHolonomicPose().getY(), m_chassis.getFusedPose().getRotation())));
    }
    public static CommandBase resetOdometryPos(SwerveChassis m_chassis, PathPlannerTrajectory trajectory, Rotation2d rot) {
        return new InstantCommand(() -> m_chassis.resetPose(new Pose2d(trajectory.getInitialHolonomicPose().getX(), trajectory.getInitialHolonomicPose().getY(), rot)));
    }
}