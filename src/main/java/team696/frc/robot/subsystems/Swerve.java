package team696.frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team696.frc.lib.Util;
import team696.frc.lib.Swerve.SwerveDriveSubsystem;
import team696.frc.lib.Swerve.SwerveModule;
import team696.frc.robot.Constants;

public class Swerve extends SwerveDriveSubsystem {
  private static Swerve m_Swerve;

  public static Swerve get() {
    if (m_Swerve == null) {
      m_Swerve = new Swerve();
    }
    return m_Swerve;
  }
  
  private Swerve() {
    super();

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
  public void onUpdate() { 

    Logger.recordOutput("Pose", getPose()); 

    Constants.Field.sim.setRobotPose(getPose());
  }

  @Override 
  public void simulationPeriodic() { 
    
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.DEBUG) {
      for(SwerveModule mod : getModules()){
        builder.addDoubleProperty("Mod " + mod.moduleNumber + " Cancoder", ()->mod.getCANCoderAngle().getRotations(), null);
        builder.addDoubleProperty("Mod " + mod.moduleNumber + " Motor", ()->mod.getState().angle.getRotations(), null);
      }
      builder.addDoubleProperty("Gyro", ()->getYaw().getDegrees(), null);
      builder.addDoubleProperty("DistToSpeaker",()->getDistToSpeaker(),null);
    }

    SmartDashboard.putData("Field", Constants.Field.sim);
  }
}
