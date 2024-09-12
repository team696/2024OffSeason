package team696.frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team696.frc.robot.util.Constants;
import team696.frc.robot.util.LLCamera;
import team696.frc.robot.util.PVCamera;
import team696.frc.robot.util.SwerveModule;
import team696.frc.robot.util.Util;

public class Swerve extends SubsystemBase {
  private static Swerve m_Swerve;

  private Pigeon2 m_Pigeon; 

  public SwerveModulePosition[] m_swervePositions = new SwerveModulePosition[4];
  private SwerveDrivePoseEstimator m_poseEstimator;

  private Rotation2d yawOffset = new Rotation2d(0);

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
    m_Pigeon.getConfigurator().apply(Constants.configs.swerve.pigeon);

    m_poseEstimator = new SwerveDrivePoseEstimator(Constants.swerve.kinematics, getYaw(), m_swervePositions, new Pose2d(0,0,new Rotation2d(0)), VecBuilder.fill(0.1, 0.1, 0.01), VecBuilder.fill(0.3, 0.3, 0.6)); 
  
    zeroYaw();
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-1 * m_Pigeon.getAngle()); 
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void zeroYaw() {
    yawOffset = getYaw();
    resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(Util.getAlliance() == DriverStation.Alliance.Red ? 180 : 0) ));
  }

  public void updateYawOffset() {
    yawOffset = getPose().getRotation().minus(getYaw());
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
                                getYaw().plus(yawOffset).rotateBy(Rotation2d.fromDegrees( (Util.getAlliance() == Alliance.Red ? 180 : 0) ))    /* getPose().getRotation().rotateBy(Rotation2d.fromDegrees( (Util.getAlliance() == Alliance.Red ? 180 : 0) )) */
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
    
    for(SwerveModule mod : Constants.swerve.modules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop);
    }
  } 

  public void setModuleStates(SwerveModuleState[] desiredStates) {
      setModuleStates(desiredStates, false);
  } 

  public Rotation2d getAngleToSpeaker() {
    if (Util.getAlliance() == Alliance.Red) 
      return getVelocityAdjustedAngleToPos(Constants.Field.RED.Speaker);

    return getVelocityAdjustedAngleToPos(Constants.Field.BLUE.Speaker);
  }

  public Rotation2d getAngleToCorner() {
    if (Util.getAlliance() == Alliance.Red) 
      return getVelocityAdjustedAngleToPos(Constants.Field.RED.Corner);

    return getVelocityAdjustedAngleToPos(Constants.Field.BLUE.Corner);
  }

  public Rotation2d getAngleToPos(Translation2d position) {
      Translation2d delta = getPose().getTranslation().minus(position);
      Rotation2d rot = Rotation2d.fromRadians(Math.atan2(delta.getY(), delta.getX()));
      return rot;
  }

  public Rotation2d getVelocityAdjustedAngleToPos(Translation2d position) {
    double dist = getPose().getTranslation().getDistance(position);
    Translation2d adjustment = (new Translation2d(0, 1.0/12.0 * getRobotRelativeSpeeds().vyMetersPerSecond * dist)).rotateBy(getAngleToPos(position)).plus(getPose().getTranslation()).minus(position);
    Rotation2d rot = Rotation2d.fromRadians(Math.atan2(adjustment.getY(), adjustment.getX()));

    return rot;
  }

  public double AngleDiffFromSpeakerDeg() {
    return getPose().getRotation().minus(getAngleToSpeaker()).getDegrees();
  }

  public double getDistToSpeaker() {
    if (Util.getAlliance() == Alliance.Red) 
        return getDistToPos(Constants.Field.RED.Speaker);
   
    return getDistToPos(Constants.Field.BLUE.Speaker);
  }

  public double getDistToCorner() {
    if (Util.getAlliance() == Alliance.Red) 
        return getDistToPos(Constants.Field.RED.Corner);
    
    return getDistToPos(Constants.Field.BLUE.Corner);
  }

  public double getDistToPos(Translation2d position) {
    return getPose().getTranslation().getDistance(position);
  }

  @Override
  public void periodic() { 
    for (int i = 0; i < 4; ++i) {
      m_swervePositions[i] = Constants.swerve.modules[i].getPosition();
    }

    m_poseEstimator.update(getYaw(), m_swervePositions);

    LLCamera.get().updatePose(m_poseEstimator, getRobotRelativeSpeeds());

    Logger.recordOutput("Pose", getPose()); 

    Constants.Field.sim.setRobotPose(getPose());
  }

  @Override 
  public void simulationPeriodic() { 
    
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.DEBUG) {
      for(SwerveModule mod : Constants.swerve.modules){
        builder.addDoubleProperty("Mod " + mod.moduleNumber + " Cancoder", ()->mod.getCANCoderAngle().getRotations(), null);
        builder.addDoubleProperty("Mod " + mod.moduleNumber + " Motor", ()->mod.getState().angle.getRotations(), null);
      }
      builder.addDoubleProperty("Gyro", ()->getYaw().getDegrees(), null);
      builder.addDoubleProperty("DistToSpeaker",()->getDistToSpeaker(),null);
    }

    SmartDashboard.putData("Field", Constants.Field.sim);
  }
}
