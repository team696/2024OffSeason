package team696.frc.lib.Cameras;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import team696.frc.lib.PLog;

public abstract class BaseCam {
    public class AprilTagResult {
        public Pose2d pose;
        public double time;

        public double distToTag;
        public int tagCount;

        public double ambiguity;

        public AprilTagResult(Pose2d pose, double time, double distToTag, int tagCount, double ambiguity) {
            this.pose = pose;
            this.time = time;
            this.distToTag = distToTag;
            this.tagCount = tagCount;
            this.ambiguity = ambiguity;
        }
    }

    public abstract Optional<AprilTagResult> getEstimate();

    Vector<N3> stdDeviations = VecBuilder.fill(0.7, 0.7, 2);

    public void setStdDeviations(double x, double y, double r) {
        stdDeviations = VecBuilder.fill(x,y,r);
    }

    @FunctionalInterface
    public static interface addVisionEstimate {
        void accept(Pose2d p, double d, Vector<N3> v);
    }

    @FunctionalInterface
    public static interface acceptEstimate {
        boolean test(AprilTagResult latestResult);
    }

    // eventually switch this to taking in a addVisionEstimate
    public boolean addVisionEstimate(addVisionEstimate addVisionMeasurement, acceptEstimate checkEstimation) {
        Optional<AprilTagResult> oEstimation = this.getEstimate();
        
        if(oEstimation.isPresent()) {
            AprilTagResult estimation = oEstimation.get();
            try {
                if (!checkEstimation.test(estimation)) {
                    return false;
                }
            } catch (Exception e) {
                PLog.fatalException("LimeLightCam", e.getMessage(), e);
            }
            addVisionMeasurement.accept(
                estimation.pose,
                estimation.time,
                stdDeviations);
            return true;
        }
        return false;
    }
    public synchronized boolean addVisionEstimate(addVisionEstimate addVisionMeasurement) { 
        return addVisionEstimate(addVisionMeasurement, (latestResult)->{return true;});
    }
}
