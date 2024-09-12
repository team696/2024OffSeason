// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
        for (int port = 5800; port <= 5809; port++) { // Need to do this for each limelight, check documentation
            PortForwarder.add(port, "limelight.local", port);
            PortForwarder.add(port, "notechaser.local", port);
        }

        int[] validIDs = {5,6}; //Only look at these tags
        LimelightHelpers.SetFiducialIDFiltersOverride("notechaser", validIDs);   
    }

    /**
     * Uses the limelight to update the pose of the robot
     * @param estimator Pose estimator, will be mutated by this function
     * @param vel Current Chassis speeds
     */
    public void updatePose(
        SwerveDrivePoseEstimator estimator,
        ChassisSpeeds vel
    ) {
        LimelightHelpers.SetRobotOrientation("limelight", Swerve.get().getPose().getRotation().getDegrees(),0,0,0,0,0);
        LimelightHelpers.SetRobotOrientation("notechaser", Swerve.get().getPose().getRotation().getDegrees(),0,0,0,0,0);
        
        LimelightHelpers.PoseEstimate amp_mt=LimelightHelpers.getBotPoseEstimate_wpiBlue("notechaser");
        estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.007, 0.007, 0.01));
        if(amp_mt != null && amp_mt.tagCount != 0)  {
           estimator.addVisionMeasurement(amp_mt.pose, amp_mt.timestampSeconds);
            return;
        }
        
        
        
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

    /**
     * Returns the ideal Yaw for absorbing the game peice
     */
    double getDesiredYawforGamePiece(){
        NetworkTable notechaser=NetworkTableInstance.getDefault().getTable("notechaser");
        boolean tv=notechaser.getValue("tv").getBoolean();
        double tx=notechaser.getValue("tx").getDouble();
        return tv?Swerve.get().getPose().getRotation().getDegrees()+tx:0;
        
    }


}
