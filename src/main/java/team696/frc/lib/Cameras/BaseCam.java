package team696.frc.lib.Cameras;

import java.util.Optional;
import java.util.function.Predicate;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
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

    public abstract Optional<AprilTagResult> getEstimate(Rotation2d curYaw);

    Vector<N3> stdDeviations = VecBuilder.fill(0.7, 0.7, 2);

    public void setStdDeviations(double x, double y, double r) {
        stdDeviations = VecBuilder.fill(x,y,r);
    }

    /* checkEstimation Used to filter out unwanted results, can use cam.latestResult to filter 
     * 
     * example usage: testCam.updateEstimator(m_poseEstimator, ()->{ if (testCam.latestResult.distToTag > 2) return false; return true;});
     * 
    */
    public boolean updateEstimator(Rotation2d curYaw, SwerveDrivePoseEstimator estimator, Predicate<AprilTagResult> checkEstimation) {
        Optional<AprilTagResult> oEstimation = this.getEstimate(curYaw);
        
        if(oEstimation.isPresent()) {
            AprilTagResult estimation = oEstimation.get();
            try {
                if (!checkEstimation.test(estimation)) {
                    return false;
                }
            } catch (Exception e) {
                PLog.fatalException("LimeLightCam", e.getMessage(), e);
            }
            estimator.setVisionMeasurementStdDevs(stdDeviations);
            estimator.addVisionMeasurement(
                estimation.pose,
                estimation.time);
            return true;
        }
        return false;
    }

    public boolean updateEstimator(Rotation2d curYaw, SwerveDrivePoseEstimator estimator) { // default behavior of cutting off bad readings, a lot to fix, don't care
        return updateEstimator(curYaw, estimator, (latestResult)-> {
            if (latestResult.ambiguity > 0.17) return false; // Too Ambiguous, Ignore
               /*  if (Math.abs(vel.omegaRadiansPerSecond) > 1.5) return; // Rotating too fast, ignore
                if (
                    Math.sqrt(
                        vel.vxMetersPerSecond * vel.vxMetersPerSecond + vel.vyMetersPerSecond * vel.vyMetersPerSecond
                    ) >
                    Constants.swerve.maxSpeed * 0.6
                ) return; // Moving Too fast, ignore*/
                double deviationRatio;
                if (latestResult.ambiguity < 1 / 100.0) {
                    deviationRatio = 1 / 100.0; // Tag estimation very good -> Use it
                } else {
                    deviationRatio = Math.pow(latestResult.distToTag, 2) / 2; // Trust Less With Distance
                }
                if(DriverStation.isAutonomousEnabled()) {
                    if (latestResult.distToTag > 4.) return false; // Tag Too far, Ignore --> comment for know becuase deviation ratio sort of fixes this.
                
                    deviationRatio *= 2;
                }
                Matrix<N3, N1> deviation = VecBuilder.fill(
                    deviationRatio,
                    deviationRatio,
                    deviationRatio
                );
                estimator.setVisionMeasurementStdDevs(deviation);
                return true;
        });
    }
}
