package team696.frc.lib.Camera;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/* Maybe in future remove limelightHelpers Dependency,
    really just copy pasting but I'm lazy */

public class LimeLightCam extends BaseCam {
    public String name = "";
    public static int LimeLightCount = 0;
    
    private NetworkTable _ntTable;
    
    public LimeLightCam(String name, int[] TagsToCheck) {
        this.name = name;

        if(TagsToCheck.length > 0) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, TagsToCheck); 
        }

        for (int port = 5800; port <= 5809; port++) { 
            PortForwarder.add(port + 10 * LimeLightCount, String.format("%s.local", this.name), port);
        }

        _ntTable = NetworkTableInstance.getDefault().getTable(name);

        _ntTable.getEntry("ledMode").setNumber(1); // Example of how to use -> This turns off LEDS on the front

        LimeLightCount++;
    }

    public LimeLightCam(String name) {
        this(name, new int[] {});
    }

    public int targetCount() {
        double[] t2d = _ntTable.getEntry("t2d").getDoubleArray(new double[0]);
        if(t2d.length == 17)
        {
          return (int)t2d[1];
        }
        return 0;    
    }

    public boolean hasTargets() {
        return targetCount() > 0;
    }

    public double tX() {
        return -1 * _ntTable.getEntry("tx").getDouble(0);
    }

    public void SetRobotOrientation(Rotation2d curYaw) {
        LimelightHelpers.SetRobotOrientation(name, curYaw.getDegrees(),0,0,0,0,0);
    }

    public Optional<AprilTagResult> getEstimate() {
        LimelightHelpers.PoseEstimate latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (latestEstimate == null) return Optional.empty();

        if (latestEstimate.tagCount == 0) return Optional.empty();

        return Optional.of(
            new AprilTagResult(latestEstimate.pose, 
                latestEstimate.timestampSeconds, 
                latestEstimate.avgTagDist, 
                latestEstimate.tagCount,
                latestEstimate.rawFiducials[0].ambiguity)); // Probably not the best but good enough for now           
    }
}
