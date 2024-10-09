// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.test;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TestStateEstimation {
    private static TestStateEstimation _estimate;

    public static TestStateEstimation get() {
        if (_estimate == null) {
            _estimate = new TestStateEstimation();
        }
        return null;//_estimate; Force a crash so people don't use this useless 
    }

    private static double time = 0;

    private final static ExtendedKalmanFilter<N3, N3, N3> filter = 
        new ExtendedKalmanFilter<>(Nat.N3(), 
                                    Nat.N3(), 
                                    Nat.N3(), 
                                    (x, u)->u, 
                                    (x, u)->x, 
                                    VecBuilder.fill(0.01, 0.01, 0.01), 
                                    VecBuilder.fill(0.01, 0.01, 0.01), 
                                    0.02);
    static {
        filter.setXhat(VecBuilder.fill(0,0,0));

    }

    private static SwerveModulePosition[] previousStates = { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

    private static Rotation2d previousAngle = new Rotation2d();

    public void updateOdometry(SwerveModulePosition[] newStates, SwerveDriveKinematics kine, Rotation2d newAngle) {
        Twist2d twist = kine.toTwist2d(
            new SwerveModulePosition(newStates[0].distanceMeters - previousStates[0].distanceMeters, newStates[0].angle.minus(previousStates[0].angle)),
            new SwerveModulePosition(newStates[1].distanceMeters - previousStates[1].distanceMeters, newStates[1].angle.minus(previousStates[1].angle)),
            new SwerveModulePosition(newStates[2].distanceMeters - previousStates[2].distanceMeters, newStates[2].angle.minus(previousStates[2].angle)),
            new SwerveModulePosition(newStates[3].distanceMeters - previousStates[3].distanceMeters, newStates[3].angle.minus(previousStates[3].angle))
        );

        twist.dtheta = newAngle.minus(previousAngle).getRadians();

        Pose2d delta = (new Pose2d()).exp(twist);

        previousStates = newStates;
        previousAngle = newAngle;

        double dT = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();
        filter.predict(VecBuilder.fill(delta.getX() / dT, delta.getY() / dT, delta.getRotation().getRadians() / dT), dT);
    }

    public void updateWithVision(Pose2d pose) {
        filter.correct(VecBuilder.fill(0,0,0), VecBuilder.fill(pose.getX(), pose.getY(), pose.getRotation().getRadians()));
    }

    public Pose2d getCurrentEstimate() {
        return new Pose2d(filter.getXhat(0), filter.getXhat(1), Rotation2d.fromRadians(filter.getXhat(2)));
    }
}

