package team696.frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team696.frc.lib.Log.PLog;
import team696.frc.robot.commands.GroundIntake;
import team696.frc.robot.commands.ManualShot;
import team696.frc.robot.commands.Rotate;
import team696.frc.robot.commands.Shoot;
import team696.frc.robot.subsystems.Swerve;

public class Auto {
    public static Auto m_instance;
    private Swerve m_swerve;

    private final SendableChooser<Command> autoChooser;

    private Auto () {
        m_swerve = Swerve.get();
        
        final HolonomicPathFollowerConfig FollowConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            Constants.swerve.maxSpeed, // Max module speed, in m/s
            Math.sqrt(Math.pow(Constants.swerve.wheelX / 2, 2) + Math.pow(Constants.swerve.wheelY / 2, 2)), // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        );

        AutoBuilder.configureHolonomic(
            m_swerve::getPose, 
            m_swerve::resetPose, 
            m_swerve::getRobotRelativeSpeeds,
            m_swerve::Drive, 
            FollowConfig,
            () -> {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            m_swerve
        );

        NamedCommands.registerCommand("ShootIntakeShoot", ((((new Shoot()).andThen(new GroundIntake())).andThen(new Shoot())).asProxy()) );
        NamedCommands.registerCommand("IntakeShoot", (((new GroundIntake()).andThen(new Shoot())).asProxy()) );
        NamedCommands.registerCommand("ShootFree", ((new Shoot()).asProxy()) );
        NamedCommands.registerCommand("Shoot", (new Rotate()).andThen( (new Shoot()).asProxy()) );
        NamedCommands.registerCommand("Intake", (new GroundIntake()).asProxy());
        NamedCommands.registerCommand("Drop", (new ManualShot(new Constants.shooter.state(0, 2500, 2500))).asProxy());
        NamedCommands.registerCommand("Subwoofer", (new ManualShot(new Constants.shooter.state(4.7, 3800, 3900))));

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            Constants.Field.sim.getObject("Target").setPose(pose);
            Logger.recordOutput("Auto/Desired", pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            Constants.Field.sim.getObject("Path").setPoses(poses);
        });

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        autoChooser.onChange((command)-> {
            visualize();
        });
    }

    public static void Initialize(){
        if (m_instance != null) throw new RuntimeException ("Can't Initialize Twice!");

        m_instance = new Auto();
    }

    public static Auto get(){
        if (m_instance == null) throw new RuntimeException ("Please Initialize First!");

        return m_instance;
    }

    public static Command Selected() {
        if (m_instance == null) return new WaitCommand(0);

        return m_instance.autoChooser.getSelected();
    }

    public static Command PathFind(Pose2d end) {
        return AutoBuilder.pathfindToPose(end, new PathConstraints(1, 1, Math.PI,Math.PI));
    }

    public Command PathFindToAutoBeginning() {
        Pose2d initialPose;
        try {
            List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName());

            if (paths.size() <= 0) return new WaitCommand(0);

            initialPose = paths.get(0).getPreviewStartingHolonomicPose();
        } catch (Exception e) {
            PLog.fatalException("Auto", "Failed To Find Path", e);
            return new WaitCommand(0);
        }

        return AutoBuilder.pathfindToPose(initialPose, new PathConstraints(1, 1, Math.PI, Math.PI));
    }

    public void visualize() {
        List<PathPlannerPath> paths;
        List<Pose2d> pathPoses = new ArrayList<Pose2d>();
        try {
            paths = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName());
        } catch (Exception e) {
            PLog.fatalException("Auto", "Failed To Find Path", e);
            return;
        }
        for (int i = 0; i < paths.size(); i++) {
            PathPlannerPath path = paths.get(i);
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                path = path.flipPath();
            pathPoses.addAll(path.getPathPoses());
        }
        Constants.Field.sim.getObject("traj").setPoses(pathPoses);
    }
}