package team696.frc.lib.Cameras;

import java.util.Optional;
import java.util.concurrent.Callable;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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

    public AprilTagResult latestResult;

    public abstract Optional<AprilTagResult> getEstimate();

    Vector<N3> stdDeviations = VecBuilder.fill(0.7, 0.7, 2);

    public void setStdDeviations(double x, double y, double r) {
        stdDeviations = VecBuilder.fill(x,y,r);
    }

    /* checkEstimation Used to filter out unwanted results, can use cam.latestResult to filter 
     * 
     * example usage: testCam.updateEstimator(m_poseEstimator, ()->{ if (testCam.latestResult.distToTag > 2) return false; return true;});
     * 
    */
    public boolean updateEstimator(SwerveDrivePoseEstimator estimator, Callable<Boolean> checkEstimation) {
        Optional<AprilTagResult> oEstimation = this.getEstimate();
        
        if(oEstimation.isPresent()) {
            this.latestResult = oEstimation.get();
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

    public boolean updateEstimator(SwerveDrivePoseEstimator estimator) { // default behavior of cutting off bad readings, a lot to fix, don't care
        return updateEstimator(estimator, ()-> {
            if (this.latestResult.ambiguity > 0.17) return false; // Too Ambiguous, Ignore
               /*  if (Math.abs(vel.omegaRadiansPerSecond) > 1.5) return; // Rotating too fast, ignore
                if (
                    Math.sqrt(
                        vel.vxMetersPerSecond * vel.vxMetersPerSecond + vel.vyMetersPerSecond * vel.vyMetersPerSecond
                    ) >
                    Constants.swerve.maxSpeed * 0.6
                ) return; // Moving Too fast, ignore*/
                double deviationRatio;
                if (this.latestResult.ambiguity < 1 / 100.0) {
                    deviationRatio = 1 / 100.0; // Tag estimation very good -> Use it
                } else {
                    deviationRatio = Math.pow(
                        this.latestResult.distToTag,
                        2
                    ) /
                    2; // Trust Less With Distance
                }
                if(DriverStation.isAutonomousEnabled()) {
                    if (this.latestResult.distToTag > 4.) return false; // Tag Too far, Ignore --> comment for know becuase deviation ratio sort of fixes this.
                
                    deviationRatio *= 2;
                }
                Matrix<N3, N1> deviation = VecBuilder.fill(
                    deviationRatio,
                    deviationRatio,
                    2 * deviationRatio
                );
                estimator.setVisionMeasurementStdDevs(deviation);
                return true;
        });
    }
}
