package team696.frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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

import team696.frc.lib.Log.PLog;
import team696.frc.robot.Robot;
import team696.frc.robot.subsystems.Swerve;

public class PVCamera {

    private class e_cam {

        @SuppressWarnings("unused")
        public String name;

        private PhotonCamera m_PCamera;
        private PhotonPoseEstimator m_Estimator;

        private PhotonCameraSim m_sim;

        public Transform3d position;

        public e_cam(String name, String cam_name, Transform3d pos) {
            this.name = name;
            this.position = pos;

            m_PCamera = new PhotonCamera(cam_name);
            m_Estimator = new PhotonPoseEstimator(
                null,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                m_PCamera,
                position
            );
            m_Estimator.setMultiTagFallbackStrategy(
                PoseStrategy.LOWEST_AMBIGUITY
            );

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

    private List<e_cam> m_EstimationCameras;

    private VisionSystemSim visionSim;

    private PhotonCamera m_NoteCamera;

    public static synchronized PVCamera get() {
        if (m_Camera == null) {
            m_Camera = new PVCamera();
        }
        return m_Camera;
    }

    private PVCamera() {
        m_EstimationCameras = new ArrayList<e_cam>();

        try {
            m_atLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2024Crescendo.m_resourceFile
            );
        } catch (Exception e) {
            PLog.fatalException(
                "Camera",
                "Failed to load april tag layout.",
                e
            );
        }

        visionSim = new VisionSystemSim("visionSim");
        visionSim.addAprilTags(m_atLayout);

        m_EstimationCameras.add(
            new e_cam(
                "Shooter",
                "B",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(-6.516),
                        Units.inchesToMeters(4.9),
                        Units.inchesToMeters(8.75)
                    ),
                    new Rotation3d(0, Math.toRadians(-23), -Math.PI)
                )
            )
        );

        for (e_cam camera : m_EstimationCameras) {
            camera.setLayout(m_atLayout);
        }

        m_NoteCamera = new PhotonCamera("placeHolder");

        if (Robot.isReal()) {
            PortForwarder.add(5800, "photonvision.local", 5800);
            PLog.info(
                "Photonvision",
                "Initialized: Dashboard open at http://photonvision.local:5800"
            );
        } else {
            PLog.info(
                "Photonvisin sim",
                "Initialized Sim: Open at http://localhost:1181"
            );
        }
    }

    /** Update Swerve Drive Pose Estimator */
    public void updatePose(
        SwerveDrivePoseEstimator estimator,
        ChassisSpeeds vel
    ) {
        if (RobotBase.isSimulation()) return;

        for (AprilTag tag : m_atLayout.getTags()) {
            Constants.Field.sim
                .getObject(tag.ID + " Real Position")
                .setPose(tag.pose.toPose2d());
        }

        for (e_cam camera : m_EstimationCameras) {
            if (!camera.m_PCamera.isConnected()) {
                PLog.unusual("Camera", camera.name + " Not Found!");
                continue;
            }

            Constants.Field.sim
                .getObject(camera.name)
                .setPose(
                    new Pose2d(
                        Swerve.get()
                            .getPose()
                            .getTranslation()
                            .plus(
                                camera.position
                                    .getTranslation()
                                    .toTranslation2d()
                                    .rotateBy(
                                        Swerve.get().getPose().getRotation()
                                    )
                            ),
                        new Rotation2d()
                    )
                );

            Optional<EstimatedRobotPose> estimation = camera.latest();
            if (estimation.isPresent()) {
                List<PhotonTrackedTarget> targets = estimation.get()
                    .targetsUsed;
                targets.sort(AmbiguityCompare);
                if (Constants.DEBUG) {
                    for (PhotonTrackedTarget t : targets) { // Shows where tags are believed to be based on robot pose.
                        Transform3d targetTransform = t.getBestCameraToTarget();
                        Pose2d target = new Pose2d(
                            targetTransform
                                .getTranslation()
                                .rotateBy(camera.position.getRotation())
                                .toTranslation2d(),
                            targetTransform.getRotation().toRotation2d()
                        );
                        Transform2d balls = new Transform2d(
                            target.getTranslation(),
                            target.getRotation()
                        );
                        Pose2d tagToCam = Swerve.get()
                            .getPose()
                            .transformBy(balls);
                        Constants.Field.sim
                            .getObject(String.valueOf(t.getFiducialId()))
                            .setPose(
                                tagToCam.plus(
                                    new Transform2d(
                                        camera.position
                                            .getTranslation()
                                            .toTranslation2d()
                                            .rotateBy(new Rotation2d(Math.PI)),
                                        camera.position
                                            .getRotation()
                                            .toRotation2d()
                                    )
                                )
                            );
                    }
                }
                PhotonTrackedTarget bestTarget = targets.get(0);
                if (bestTarget.getPoseAmbiguity() > 0.17) return; // Too Ambiguous, Ignore
                if (Math.abs(vel.omegaRadiansPerSecond) > 1.5) return; // Rotating too fast, ignore
                if (
                    Math.sqrt(
                        vel.vxMetersPerSecond * vel.vxMetersPerSecond + vel.vyMetersPerSecond * vel.vyMetersPerSecond
                    ) >
                    Constants.swerve.maxSpeed * 0.6
                ) return; // Moving Too fast, ignore
                double deviationRatio;
                if (bestTarget.getPoseAmbiguity() < 1 / 100.0) {
                    deviationRatio = 1 / 100.0; // Tag estimation very good -> Use it
                } else {
                    deviationRatio = Math.pow(
                        bestTarget
                            .getBestCameraToTarget()
                            .getTranslation()
                            .getNorm(),
                        2
                    ) /
                    2; // Trust Less With Distance
                }
                if(DriverStation.isAutonomousEnabled()) {
                    if (bestTarget.getBestCameraToTarget().getTranslation().getNorm() > 4.) return; // Tag Too far, Ignore --> comment for know becuase deviation ratio sort of fixes this.
                
                    deviationRatio *= 2;
                }
                Matrix<N3, N1> deviation = VecBuilder.fill(
                    deviationRatio,
                    deviationRatio,
                    2 * deviationRatio
                );
                estimator.setVisionMeasurementStdDevs(deviation);
                estimator.addVisionMeasurement(
                    estimation.get().estimatedPose.toPose2d(),
                    estimation.get().timestampSeconds
                );
            }
        }
    }

    public void simPeriodic(Pose2d pose) {
        visionSim.update(pose);
    }

    Comparator<PhotonTrackedTarget> AmbiguityCompare = new Comparator<PhotonTrackedTarget>() {
        @Override
        public int compare(PhotonTrackedTarget o1, PhotonTrackedTarget o2) {
            return (int) ((o1.getPoseAmbiguity() - o2.getPoseAmbiguity()) *
                100);
        }
    };

    public double getBestTargetYaw() {
        PhotonTrackedTarget best = m_NoteCamera
            .getLatestResult()
            .getBestTarget();
        
        if (best != null) {
            double yaw = best.getYaw();

            return yaw;
        }

        return 0;
    }
}
