package team696.frc.lib.Swerve;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team696.frc.lib.Util;

// Must call super() at the top of the constructor -> maybe theres a better way, idk

public abstract class SwerveDriveSubsystem extends SubsystemBase {
    
    private SwerveModulePosition[] _swervePositions = new SwerveModulePosition[4];
    private SwerveDrivePoseEstimator _poseEstimator;

    private SwerveModule[] _modules;
    private SwerveDriveKinematics _kinematics;

    private Pigeon2 _pigeon; 

    private Rotation2d yawOffset = new Rotation2d(0);

    public SwerveDriveSubsystem() {
		SwerveModule frontLeft = new SwerveModule(0, SwerveConfigs.Mod0);
		SwerveModule frontRight = new SwerveModule(1,SwerveConfigs.Mod1);
		SwerveModule backLeft = new SwerveModule(2,  SwerveConfigs.Mod2);
		SwerveModule backRight = new SwerveModule(3, SwerveConfigs.Mod3);
		_modules = new SwerveModule[]{ frontLeft, frontRight, backLeft, backRight };

        _kinematics = new SwerveDriveKinematics(SwerveConstants.modPositions);

        for (int i = 0; i < 4; ++i) {
            _swervePositions[i] = _modules[i].getPosition();
        }

        _pigeon = new Pigeon2(0);
        _pigeon.getConfigurator().apply(SwerveConfigs.pigeon);

        _poseEstimator = new SwerveDrivePoseEstimator(_kinematics, getYaw(), _swervePositions, new Pose2d(0,0,new Rotation2d(0)), VecBuilder.fill(0.1, 0.1, 0.01), VecBuilder.fill(0.3, 0.3, 0.6)); 
    
        zeroYaw();
    }

    public Pose2d getPose() {
        return _poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose) {
        _poseEstimator.resetPosition(getYaw(), _swervePositions, newPose);
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public void updateYawOffset() {
        yawOffset = getPose().getRotation().minus(getYaw());
    }

    // Called periodically -> 50 Hz
    public abstract void onUpdate();

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(-1 * _pigeon.getAngle()); 
    }

    public void zeroYaw() {
        yawOffset = getYaw();
        resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(Util.getAlliance() == DriverStation.Alliance.Red ? 180 : 0) ));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return _kinematics.toChassisSpeeds(getStates());
    }

    private SwerveModuleState[] getStates() { 
        SwerveModuleState[] states = new SwerveModuleState[4]; 
        for(SwerveModule mod : _modules) { 
        states[mod.moduleNumber] = mod.getState(); 
        } 
        return states; 
    }

    public void Drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            _kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getYaw().plus(yawOffset).rotateBy(Rotation2d.fromDegrees( (Util.getAlliance() == Alliance.Red ? 180 : 0) )) 
                            ) : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation));

      setModuleStates(swerveModuleStates, isOpenLoop);
    }

    public void Drive(ChassisSpeeds c) {
        SwerveModuleState[] swerveModuleStates = _kinematics.toSwerveModuleStates(c);
        setModuleStates(swerveModuleStates);
    } 
    
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : _modules) {
          mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop);
        }
    } 
    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, false);
    } 

    public SwerveModule[] getModules() {
        return _modules;
    }

    @Override
    public final void periodic() {
        if (DriverStation.isDisabled()) {
            this.updateYawOffset();
        }

        for (int i = 0; i < 4; ++i) {
            _swervePositions[i] = _modules[i].getPosition();
        }
      
        _poseEstimator.update(getYaw(), _swervePositions);
      
        onUpdate();
    }
}
