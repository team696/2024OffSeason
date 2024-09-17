// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.lib.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class SwerveDriveState {
    public Pose2d pose;
    public ChassisSpeeds robotRelativeSpeeds;
    public double timeStamp;

    public SwerveDriveState(Pose2d pose, ChassisSpeeds speeds, double time) {
        update(pose, speeds, time);
    }

    public SwerveDriveState(SwerveDriveState other) {
        this.pose = other.pose;
        this.robotRelativeSpeeds = other.robotRelativeSpeeds;
        this.timeStamp = other.timeStamp;
    }

    public SwerveDriveState() {
        this(new Pose2d(), new ChassisSpeeds(), 0);
    }

    public double velocity() {
        return Math.sqrt(robotRelativeSpeeds.vxMetersPerSecond * robotRelativeSpeeds.vxMetersPerSecond + robotRelativeSpeeds.vyMetersPerSecond * robotRelativeSpeeds.vyMetersPerSecond);
    }

    public double angularVelocity() {
        return Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond);
    }

    public void update(Pose2d pose, ChassisSpeeds speeds, double time) {
        this.pose = pose;
        this.robotRelativeSpeeds = speeds;
        this.timeStamp = time;
    }
}
