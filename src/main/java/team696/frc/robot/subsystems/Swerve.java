package team696.frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team696.frc.robot.util.PVCamera;
import team696.frc.robot.Robot;
import team696.frc.robot.util.Constants;
import team696.frc.robot.util.SwerveModule;

public class Swerve extends SubsystemBase {
  private static Swerve m_Swerve;

  private Pigeon2 m_Pigeon; 

  private SwerveModulePosition[] m_swervePositions = new SwerveModulePosition[4];
  private SwerveDrivePoseEstimator m_poseEstimator;

  private SimSwerveDrivetrain m_SimSwerveDrivetrain;

  public static Swerve get() {
    if (m_Swerve == null) {
      m_Swerve = new Swerve();
    }
    return m_Swerve;
  }
  
  private Swerve() {
    for (int i = 0; i < 4; ++i) {
      m_swervePositions[i] = Constants.swerve.modules[i].getPosition();
    }

    m_Pigeon = new Pigeon2(0);
    m_Pigeon.getConfigurator().apply(Constants.CONFIGS.swerve_Pigeon);

    m_poseEstimator = new SwerveDrivePoseEstimator(Constants.swerve.kinematics, getYaw(), m_swervePositions, new Pose2d(0,0,new Rotation2d(0)), VecBuilder.fill(0.1, 0.1, 0.01), VecBuilder.fill(0.3, 0.3, 0.6)); 
  
    zeroYaw();

    if (Robot.isSimulation()) {
      m_SimSwerveDrivetrain = new SimSwerveDrivetrain(Constants.swerve.modPositions, m_Pigeon, Constants.CONFIGS.drivetrain, Constants.CONFIGS.Mods);
    }
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-1 * m_Pigeon.getAngle()); 
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void zeroYaw() {
    m_Pigeon.reset();
    resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees((DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0) : 0))));
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getYaw(), m_swervePositions, pose);
  }

  public void resetPose(){
    resetPose(new Pose2d());
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.swerve.kinematics.toChassisSpeeds(getStates());
  }

  private SwerveModuleState[] getStates() { 
    SwerveModuleState[] states = new SwerveModuleState[4]; 
    for(SwerveModule mod : Constants.swerve.modules) { 
      states[mod.moduleNumber] = mod.getState(); 
    } 
    return states; 
  }

  public void Drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.swerve.kinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getPose().getRotation().rotateBy(Rotation2d.fromDegrees((DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0) : 0)))
                            ) : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation));

      setModuleStates(swerveModuleStates, isOpenLoop);
  }

  public void Drive(ChassisSpeeds c) {
    SwerveModuleState[] swerveModuleStates = Constants.swerve.kinematics.toSwerveModuleStates(c);
    setModuleStates(swerveModuleStates);
  } 

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.swerve.maxSpeed);
    
    for(SwerveModule mod : Constants.swerve.modules){
        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  } 

  public void setModuleStates(SwerveModuleState[] desiredStates) {
      setModuleStates(desiredStates, false);
  } 

  @Override
  public void periodic() { 
    for (int i = 0; i < 4; ++i) {
      m_swervePositions[i] = Constants.swerve.modules[i].getPosition();
    }

    Twist2d last_velocity_measured = new Twist2d(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond, getRobotRelativeSpeeds().omegaRadiansPerSecond);

    m_poseEstimator.update(getYaw(), m_swervePositions);

    PVCamera.get().updatePose(m_poseEstimator, last_velocity_measured);

    Logger.recordOutput("Pose", getPose());

    Constants.Field.sim.setRobotPose(getPose());
  }

  @Override 
  public void simulationPeriodic() {
    m_SimSwerveDrivetrain.update(0.02, RobotController.getBatteryVoltage(), Constants.swerve.sim_modules);
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.DEBUG) {
      for(SwerveModule mod : Constants.swerve.modules){
        builder.addDoubleProperty("Mod " + mod.moduleNumber + " Cancoder", ()->mod.getCANCoderAngle().getRotations(), null);
      }
      builder.addDoubleProperty("Gyro", ()->getYaw().getDegrees(), null);
    }

    SmartDashboard.putData("Field", Constants.Field.sim);
  }
}
