package frc.robot.util;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class StateEstimator {
    private static StateEstimator m_instance;

    private UnscentedKalmanFilter<N2, N2, N2> filter;

    private TreeMap<Double, Pose2d> odom_estimation_history;
    private TreeMap<Double, Translation2d> field_to_odom_history;

    private Optional<Translation2d> initial_field_to_odom; // Pulled From First Camera Update

    private TreeMap<Double, Twist2d> velocity_history;
    private Twist2d desired_velocity;

    public static StateEstimator get() {
        if (m_instance == null) {
            m_instance = new StateEstimator();
        }
        return m_instance;
    }

    private StateEstimator() {
        initial_field_to_odom = Optional.empty();
        reset();
    }

    /** Add new odometry measurement and velocity */
    public void updateOdom(Pose2d new_pose, Twist2d measured_vel, Twist2d desired_vel) {
        filter.predict(VecBuilder.fill(0,0), 0.02);

        odom_estimation_history.put(Timer.getFPGATimestamp(), new_pose);

        if (odom_estimation_history.size() > 50) {
            odom_estimation_history.remove(odom_estimation_history.firstKey());
        }

        velocity_history.put(Timer.getFPGATimestamp(), measured_vel);
        desired_velocity = desired_vel;

        if (velocity_history.size() > 25) {
            velocity_history.remove(velocity_history.firstKey());
        }
    }

    /** Add Vision to update offsets */
    public void addVision(List<EstimatedRobotPose> cams) { // Reincorporate Previous Vision Enhancements, check 2024 Build Season Camera.java
        for (EstimatedRobotPose visionPose : cams) { 
            double visionTime = visionPose.timestampSeconds;
            Pose2d lastest_odometry = getOdometryToVehicle(visionTime);

            if (DriverStation.isAutonomousEnabled() && visionPose.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm() > 3) return; // skip during auto if we too far!

            if (Math.abs(velocity_history.lastEntry().getValue().dtheta) > 1) return; // Rotating too fast --> Bad Estimation --> 254???

            if (initial_field_to_odom.isEmpty() || DriverStation.isDisabled()) {
                field_to_odom_history.put(visionTime, visionPose.estimatedPose.toPose2d().getTranslation().minus(lastest_odometry.getTranslation()));
                initial_field_to_odom = Optional.of(field_to_odom_history.lastEntry().getValue());
                filter.setXhat(0, field_to_odom_history.lastEntry().getValue().getX());
                filter.setXhat(1, field_to_odom_history.lastEntry().getValue().getY());
            } else {
                Translation2d field_to_odom = visionPose.estimatedPose.toPose2d().getTranslation().minus(lastest_odometry.getTranslation());

                filter.correct(VecBuilder.fill(0,0), VecBuilder.fill(field_to_odom.getX(), field_to_odom.getY()));
                field_to_odom_history.put(visionTime, new Translation2d(filter.getXhat(0), filter.getXhat(1)));
            }

            if (field_to_odom_history.size() > 50) {
                field_to_odom_history.remove(field_to_odom_history.firstKey());
            }
        }
    }

    /** Resets Estimation */
    public void reset(Pose2d start_pose) {
        filter =
            new UnscentedKalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            (x, u) -> VecBuilder.fill(0.0, 0.0),
            (x, u) -> x,
            VecBuilder.fill(Math.pow(0.04, 1), Math.pow(0.04, 1)),
            VecBuilder.fill(Math.pow(0.01, 1), Math.pow(0.01, 1)), 0.02);

        odom_estimation_history = new TreeMap<Double, Pose2d>();
        odom_estimation_history.put(Timer.getFPGATimestamp(), start_pose);

        field_to_odom_history = new TreeMap<Double, Translation2d>();
        field_to_odom_history.put(Timer.getFPGATimestamp(), initial_field_to_odom.isEmpty() ? new Translation2d() : initial_field_to_odom.get());

        velocity_history = new TreeMap<Double, Twist2d>();
        velocity_history.put(Timer.getFPGATimestamp(), new Twist2d()); 

        desired_velocity = new Twist2d();
    }

    /** Resets Estimation to 0,0,0 */
    public void reset() {
        reset(new Pose2d());
    }

    /**Interpolated Odometry At Given Time */
    private Pose2d getOdometryToVehicle(double time) {
        time = Util.clamp(time, odom_estimation_history.firstKey() + 0.0001, odom_estimation_history.lastKey() - 0.0001);
        Map.Entry<Double, Pose2d> lower = odom_estimation_history.floorEntry(time);
        Map.Entry<Double, Pose2d> higher = odom_estimation_history.ceilingEntry(time);

        double lerpValue = (time - lower.getKey()) / (higher.getKey() - lower.getKey());

        return new Pose2d(Util.lerp(lerpValue, lower.getValue().getX(), higher.getValue().getY()), Util.lerp(lerpValue,lower.getValue().getY(), higher.getValue().getY()), Rotation2d.fromRadians(Util.lerp(lerpValue, lower.getValue().getRotation().getRadians(), higher.getValue().getRotation().getRadians())));
    }

    /** Avg Velocity over last 0.5s of velocity records */
    private Twist2d average_velocity() {
        Twist2d avg = new Twist2d();
        int counted = 0;
        for (Map.Entry<Double, Twist2d> velocity : velocity_history.entrySet()) {
            if (Timer.getFPGATimestamp() - velocity.getKey() > 0.5) continue; // Only Keep 0.5s latest Velocity Values

            avg.dx += velocity.getValue().dx;
            avg.dy += velocity.getValue().dy;
            avg.dtheta += velocity.getValue().dtheta;

            counted ++;
        }
        avg.dx /= counted;
        avg.dy /= counted;
        avg.dtheta /= counted;

        return avg;
    }

    /**  Last Odometry Update */
    public Pose2d getOdomPose() {
        return odom_estimation_history.lastEntry().getValue();
    }

    /**  Fused Odometry + Vision Offset */
    public Pose2d getFusedPose() {
        return odom_estimation_history.lastEntry().getValue().plus(new Transform2d(field_to_odom_history.lastEntry().getValue(), new Rotation2d()));
    }
}
