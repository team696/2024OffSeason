// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private LLCamera() {
        for (int port = 5800; port <= 5809; port++) { // Need to do this for each limelight, check documentation
            PortForwarder.add(port, "limelight.local", port);
        }

        //int[] validIDs = {3,4}; //Only look at these tags
        //LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);   
    }

    public void updatePose(
        SwerveDrivePoseEstimator estimator,
        ChassisSpeeds vel
    ) {
        LimelightHelpers.SetRobotOrientation("limelight", Swerve.get().getPose().getRotation().getDegrees(),0,0,0,0,0);
        LimelightHelpers.PoseEstimate mt2;
        //if (Util.getAlliance() == Alliance.Blue) 
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        //else 
        //    mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
        
        if (mt2 == null) return;

        if (mt2.tagCount == 0) return; // No tags

        estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,3));
        estimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
    }
}
