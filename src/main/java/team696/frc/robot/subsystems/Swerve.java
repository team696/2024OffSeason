package team696.frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team696.frc.lib.Util;
import team696.frc.lib.Cameras.LimeLightCam;
import team696.frc.lib.Swerve.SwerveConstants;
import team696.frc.lib.Swerve.SwerveDriveSubsystem;
import team696.frc.lib.Swerve.SwerveModule;
import team696.frc.robot.Constants;

public class Swerve extends SwerveDriveSubsystem {
  private static Swerve m_Swerve;

  private LimeLightCam shooterCam;
  private LimeLightCam intakeCam;
  private LimeLightCam ampCam;

  public static Swerve get() {
    if (m_Swerve == null) {
      m_Swerve = new Swerve();
    }
    return m_Swerve;
  }
  
  private Swerve() {
    shooterCam = new LimeLightCam("limelight-shooter");
    intakeCam = new LimeLightCam("limelight-note");
    ampCam = new LimeLightCam("limelight-amp");

    ampCam.setStdDeviations(0.01, 0.01, 0.01);
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

  public Rotation2d getVelocityAdjustedAngleToPos(Translation2d position) {
    double dist = distTo(position);
    Translation2d adjustment = (new Translation2d(0, 1.0/12.0 * getRobotRelativeSpeeds().vyMetersPerSecond * dist)).rotateBy(angleTo(position)).plus(getPose().getTranslation()).minus(position);
    Rotation2d rot = Rotation2d.fromRadians(Math.atan2(adjustment.getY(), adjustment.getX()));

    return rot;
  }

  public double AngleDiffFromSpeakerDeg() {
    return getPose().getRotation().minus(getAngleToSpeaker()).getDegrees();
  }

  public double getDistToSpeaker() {
    if (Util.getAlliance() == Alliance.Red) 
        return distTo(Constants.Field.RED.Speaker);
   
    return distTo(Constants.Field.BLUE.Speaker);
  }

  public double getDistToCorner() {
    if (Util.getAlliance() == Alliance.Red) 
        return distTo(Constants.Field.RED.Corner);
    
    return distTo(Constants.Field.BLUE.Corner);
  }

  public Rotation2d getAngleForNote() {
    if (intakeCam.hasTargets()) {
      return getPose().getRotation().minus(Rotation2d.fromDegrees(intakeCam.tX()));
    }
    return null;
  }

  @Override
  public void onUpdate() { 
    /* this is kinda ugly and messy, but it beats doing it inside and taking in a extra useless parameter, shit limelight shouldn't even need to do this anyway. */
    shooterCam.addVisionEstimate((x,y,r)->{shooterCam.SetRobotOrientation(getPose().getRotation());this.addVisionMeasurement(x,y,r);}, (latestResult)-> {
        if (latestResult.ambiguity > 0.17) return false; // Too Ambiguous, Ignore
        if (getState().angularVelocity() > 1.5) return false; // Rotating too fast, ignore
        if (getState().velocity() > SwerveConstants.maxSpeed * 0.6)
            return false; // Moving Too fast, ignore
        double deviationRatio;
        if (latestResult.ambiguity < 1 / 100.0) {
            deviationRatio = 0.01; // Tag estimation very good -> Use it
        } else {
          deviationRatio = Math.pow(latestResult.distToTag, 2) / 2; // Trust Less With Distance
        }
        if(DriverStation.isAutonomousEnabled()) {
            if (latestResult.distToTag > 4.) return false; // Tag Too far, Ignore --> comment for know becuase deviation ratio sort of fixes this.
        
            deviationRatio *= 2;
        }
        shooterCam.setStdDeviations(deviationRatio, deviationRatio, deviationRatio);
        return true;
    });
    ampCam.addVisionEstimate((x,y,r)->{shooterCam.SetRobotOrientation(getPose().getRotation());this.addVisionMeasurement(x,y,r);});

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
