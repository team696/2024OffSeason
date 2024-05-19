package team696.frc.robot.util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.RobotBase;
import team696.frc.lib.Log.PLog;
import team696.frc.robot.Robot;
import team696.frc.robot.subsystems.Swerve;

public class PVCamera {
    private  class cam {
        @SuppressWarnings("unused")
        public String name;

        private PhotonCamera m_PCamera;
        private PhotonPoseEstimator m_Estimator;

        private PhotonCameraSim m_sim;

        public Transform3d position;

        public cam(String name, String cam_name, Transform3d pos) {
            this.name = name;
            this.position = pos;

            m_PCamera = new PhotonCamera(cam_name);
            m_Estimator = new PhotonPoseEstimator(null, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_PCamera, position);
            m_Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            if (Robot.isSimulation()) {
                m_sim = new PhotonCameraSim(m_PCamera);

                visionSim.addCamera(m_sim, position);
            }
        }

        public void setLayout(AprilTagFieldLayout layout) {
            m_Estimator.setFieldTags(layout);
        }
        
        public Optional<EstimatedRobotPose> latest() {
            return m_Estimator.update();
        }
    }

    private static PVCamera m_Camera;

    private AprilTagFieldLayout m_atLayout; 

    private List<cam> m_Cameras;

    private VisionSystemSim visionSim;

    public static synchronized PVCamera get() {
        if (m_Camera == null) {
            m_Camera = new PVCamera();
        }
        return m_Camera;
    }
    
    private PVCamera() {
        m_Cameras = new ArrayList<cam>();

        try {
            m_atLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (Exception e) {
            PLog.fatalException("Camera", "Failed to load april tag layout.", e);
        }

        visionSim = new VisionSystemSim("visionSim");
        visionSim.addAprilTags(m_atLayout);

        m_Cameras.add(new cam("nuts", "nuts", new Transform3d()));

        for (cam camera : m_Cameras) {
            camera.setLayout(m_atLayout);
        }

        if (Robot.isReal()) {
            PortForwarder.add(5800, "photonvision.local", 5800);
            PLog.info("Photonvision", "Initialized: Dashboard open at http://photonvision.local:5800");
        } else {
            PLog.info("Photonvisin sim", "Initialized Sim: Open at http://localhost:1181");
        }
    }

    /** Update Swerve Drive Pose Estimator */
    public void updatePose(SwerveDrivePoseEstimator estimator, Twist2d vel) {
        if (RobotBase.isSimulation()) return;

        for (cam camera : m_Cameras) {
            if(!camera.m_PCamera.isConnected()) {
                PLog.unusual("Camera", camera.name + " Not Found!");
                continue;
            }

            Optional<EstimatedRobotPose> estimation = camera.latest();
            if (estimation.isPresent()) {
                List<PhotonTrackedTarget> targets = estimation.get().targetsUsed;
                targets.sort(AmbiguityCompare);
                if (Constants.DEBUG) { 
                    for (PhotonTrackedTarget t : targets) { // Shows where tags are believed to be based on robot pose.
                        Transform3d targetTransform = t.getBestCameraToTarget(); 
                        Pose2d target = new Pose2d(targetTransform.getTranslation().rotateBy(camera.position.getRotation()).toTranslation2d(), targetTransform.getRotation().toRotation2d());
                        Transform2d balls = new Transform2d(target.getTranslation(), target.getRotation());
                        Pose2d tagToCam = Swerve.get().getPose().transformBy(balls);
                        Constants.Field.sim.getObject(String.valueOf(t.getFiducialId())).setPose(tagToCam.plus(new Transform2d(camera.position.getTranslation().toTranslation2d().rotateBy(new Rotation2d(Math.PI)), camera.position.getRotation().toRotation2d())));
                    }
                }
                PhotonTrackedTarget bestTarget = targets.get(0);
                if (bestTarget.getPoseAmbiguity() > 0.13) return; // Too Ambiguous, Ignore
                if (Math.abs(vel.dtheta) > 1) return; // Rotating too fast, ignore
                if (Math.sqrt(vel.dx * vel.dx + vel.dy + vel.dy) > Constants.swerve.maxSpeed * 0.4) return; // Moving Too fast, ignore
                //if (bestTarget.getBestCameraToTarget().getTranslation().getNorm() > 4) return; // Tag Too far, Ignore --> comment for know becuase deviation ratio sort of fixes this.
                double deviationRatio; 
                if (bestTarget.getPoseAmbiguity() < 1/100.0) {
                    deviationRatio = 1/100.0; // Tag estimation very good -> Use it
                } else {
                    deviationRatio = Math.pow(bestTarget.getBestCameraToTarget().getTranslation().getNorm(),2) / 2; // Trust Less With Distance
                }
                Matrix<N3, N1> deviation = VecBuilder.fill(deviationRatio, deviationRatio, 100 * deviationRatio);
                estimator.setVisionMeasurementStdDevs(deviation);
                estimator.addVisionMeasurement(estimation.get().estimatedPose.toPose2d(), estimation.get().timestampSeconds);
            }
        }
    }

    public void simPeriodic(Pose2d pose) {
        visionSim.update(pose);
    }

    Comparator<PhotonTrackedTarget> AmbiguityCompare = new Comparator<PhotonTrackedTarget>() {
        @Override
        public int compare(PhotonTrackedTarget o1, PhotonTrackedTarget o2) {
            return (int)((o1.getPoseAmbiguity() - o2.getPoseAmbiguity()) * 100);
        }
    };
}