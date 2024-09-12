package team696.frc.lib.Cameras;

import java.util.Optional;
import java.util.concurrent.Callable;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import team696.frc.lib.LimeLight.LimelightHelpers;
import team696.frc.lib.Log.PLog;
import team696.frc.robot.subsystems.Swerve;

public class LimeLightCam {

    public class AprilTagResult {
        public Pose2d pose;
        public double time;

        public double distToTag;

        public int tagCount;

        public AprilTagResult(LimelightHelpers.PoseEstimate estimate) {
            pose = estimate.pose;
            time = estimate.timestampSeconds;

            distToTag = estimate.avgTagDist;
            tagCount = estimate.tagCount;
        }
    }

    public String name = "";
    public static int LimeLightCount = 0;
    public AprilTagResult latestResult;
    Vector<N3> stdDeviations = VecBuilder.fill(0.7, 0.7, 2);
    
    public LimeLightCam(String name, int[] TagsToCheck) {
        this.name = name;

        if(TagsToCheck.length > 0) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, TagsToCheck); 
        }

        for (int port = 5800; port <= 5809; port++) { 
            PortForwarder.add(port + 10 * LimeLightCount, String.format("%s.local", this.name), port);
        }

        LimeLightCount++;
    }

    public LimeLightCam(String name) {
        this(name, new int[] {});
    }

    boolean hasTarget() {
        return LimelightHelpers.getTargetCount(name) > 0;
    }

    double tX() {
        return LimelightHelpers.getTX(name);
    }

    public Optional<LimelightHelpers.PoseEstimate> getEstimate() {
        LimelightHelpers.SetRobotOrientation(name, Swerve.get().getPose().getRotation().getDegrees(),0,0,0,0,0);
        LimelightHelpers.PoseEstimate latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (latestEstimate == null) return Optional.empty();

        if (latestEstimate.tagCount == 0) return Optional.empty();

        return Optional.of(latestEstimate);
    }

    public void setStdDeviations(double x, double y, double r) {
        stdDeviations = VecBuilder.fill(x,y,r);
    }

    /* checkEstimation Used to filter out unwanted results, can use cam.latestResult to filter 
     * 
     * example usage: testCam.updateEstimator(m_poseEstimator, ()->{ if (testCam.latestResult.distToTag > 2) return false; return true;});
     * 
    */
    public boolean updateEstimator(SwerveDrivePoseEstimator estimator, Callable<Boolean> checkEstimation) {
        Optional<LimelightHelpers.PoseEstimate> oEstimation = this.getEstimate();
        
        if(oEstimation.isPresent()) {
            this.latestResult = new AprilTagResult(oEstimation.get());
            try {
                if (!checkEstimation.call()) {
                    return false;
                }
            } catch (Exception e) {
                PLog.fatalException("LimeLightCam", e.getMessage(), e);
            }
            estimator.setVisionMeasurementStdDevs(stdDeviations);
            estimator.addVisionMeasurement(
                this.latestResult.pose,
                this.latestResult.time);
            return true;
        }
        return false;
    }
}
