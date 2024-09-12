// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
import team696.frc.lib.LimeLight.LimelightHelpers;
import team696.frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class LLCamera {
    private static LLCamera m_Camera;

    public static synchronized LLCamera get() {
        if (m_Camera == null) {
            m_Camera = new LLCamera();
        }
        return m_Camera;
    }

    private class LimeLightHolder {
        public String name = "";
        public static int LimeLightCount = 0;
        
        public LimeLightHolder(String name, int[] TagsToCheck) {
            LimeLightCount++;

            this.name = name;

            if(TagsToCheck.length > 0) {
                LimelightHelpers.SetFiducialIDFiltersOverride(name, TagsToCheck); 
            }

            for (int port = 5800; port <= 5809; port++) { 
                PortForwarder.add(port + 10 * LimeLightCount, String.format("%s.local", this.name), port);
            }
        }

        public LimeLightHolder(String name) {
            this(name, new int[] {});
        }

        public LimeLightHolder() {
            this(String.format("limeLight%i", LimeLightCount));
        }

        boolean hasTarget() {
            return LimelightHelpers.getTargetCount(name) > 0;
        }

        double tX() {
            return LimelightHelpers.getTX(name);
        }

        Optional<LimelightHelpers.PoseEstimate> getEstimate() {
            LimelightHelpers.SetRobotOrientation(name, Swerve.get().getPose().getRotation().getDegrees(),0,0,0,0,0);
            LimelightHelpers.PoseEstimate latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

            if (latestEstimate == null) return null;

            if (latestEstimate.tagCount == 0) return null;

            return Optional.of(latestEstimate);
        }
    }

    LimeLightHolder amp;
    LimeLightHolder shooter;
    LimeLightHolder note;

    private LLCamera() {
        amp = new LimeLightHolder("amp");
        shooter = new LimeLightHolder("shooter");
        note = new LimeLightHolder("note");
    }

    public double getAngleForNote() {
        if (note.hasTarget()){
            return note.tX();
        }
        return 0;
    }

    public void updatePose(
        SwerveDrivePoseEstimator estimator,
        ChassisSpeeds vel
    ) {
        Optional<LimelightHelpers.PoseEstimate> ampEstimation = amp.getEstimate();
        
        if(ampEstimation.isPresent()) {
            estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.01,.01, .01));
            estimator.addVisionMeasurement(
                ampEstimation.get().pose,
                ampEstimation.get().timestampSeconds);
            return; // Don't even bother reading to
        }

        Optional<LimelightHelpers.PoseEstimate> shooterEstimation = shooter.getEstimate();

        if (shooterEstimation.isEmpty()) return;

        estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,3));
        estimator.addVisionMeasurement(
            shooterEstimation.get().pose,
            shooterEstimation.get().timestampSeconds);
    }
}
