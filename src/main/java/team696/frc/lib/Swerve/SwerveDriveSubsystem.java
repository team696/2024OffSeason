package team696.frc.lib.Swerve;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team696.frc.lib.Util;

public abstract class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveModulePosition[] _swervePositions = new SwerveModulePosition[4];
    private final SwerveDrivePoseEstimator _poseEstimator;

    private final SwerveModule[] _modules;
    private final SwerveDriveKinematics _kinematics;

    protected final Pigeon2 _pigeon; 

    protected Rotation2d yawOffset = new Rotation2d(0);

    protected final ReadWriteLock _stateLock;
    protected final SwerveDriveState _cachedState;

    private final odometryThread _odometryThread;

    public SwerveDriveSubsystem() {
        this._stateLock = new ReentrantReadWriteLock();
        this._cachedState = new SwerveDriveState();

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
        _pigeon.getYaw().setUpdateFrequency(100);
        _pigeon.getAngularVelocityZWorld().setUpdateFrequency(100);
        _pigeon.optimizeBusUtilization();

        _poseEstimator = new SwerveDrivePoseEstimator(_kinematics, getYaw(), _swervePositions, new Pose2d(0,0,new Rotation2d(0)), VecBuilder.fill(0.1, 0.1, 0.01), VecBuilder.fill(0.3, 0.3, 0.6)); 
    
        zeroYaw();

        _odometryThread = new odometryThread(this);
        _odometryThread.start();
    }

    public SwerveDriveState getState() {
        try {
           this._stateLock.readLock().lock();
           return this._cachedState;
        } finally {
           this._stateLock.readLock().unlock();
        }
    }

    public Pose2d getPose() {
        return getState().pose;
    }

    public void resetPose(Pose2d newPose) {
        try {
            this._stateLock.writeLock().lock();
            _poseEstimator.resetPosition(getYaw(), _swervePositions, newPose);
        } finally {
            this._stateLock.writeLock().unlock();
        }
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
        return Rotation2d.fromDegrees( MathUtil.inputModulus(_pigeon.getYaw().getValueAsDouble(),-180,180)); 
    }

    public Rotation2d latencyAdjustedYaw() {
        return Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(_pigeon.getYaw(), _pigeon.getAngularVelocityZWorld()));
    }

    public void zeroYaw() {
        yawOffset = getYaw();
        resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(Util.getAlliance() == DriverStation.Alliance.Red ? 180 : 0) ));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().robotRelativeSpeeds;
    }

    private SwerveModuleState[] getModuleStates() { 
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

    public void doNothing() {
        Drive(new ChassisSpeeds());
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

    public double distTo(Translation2d position) {
        return getPose().getTranslation().getDistance(position);
    }

    public double distTo(Pose2d position) {
        return distTo(position.getTranslation());
    }

    public Rotation2d angleTo(Translation2d position) {
        Translation2d delta = getPose().getTranslation().minus(position);
        Rotation2d rot = Rotation2d.fromRadians(Math.atan2(delta.getY(), delta.getX()));
        return rot;
    }

    public Rotation2d angleTo(Pose2d position) {
        return angleTo(position.getTranslation());
    }


    /*
     * Don't override periodic, override onUpdate()
     * 
     */
    @Override
    public final void periodic() {
        if (DriverStation.isDisabled()) {
            this.updateYawOffset();
        }
      
        onUpdate();
    }

    public void addVisionMeasurement(Pose2d visionPose, double visionTimestamp, Vector<N3> stdDeviations) {
        try {
            this._stateLock.writeLock().lock();
        
            _poseEstimator.addVisionMeasurement(visionPose, visionTimestamp, stdDeviations);
        } finally {
            this._stateLock.writeLock().unlock();
        }
    }

    class odometryThread extends Thread {
        private SwerveDriveSubsystem this0;

        odometryThread(SwerveDriveSubsystem this0) {
            this.this0 = this0;

            this.setDaemon(true);
            this.setPriority(1);
        }

        public void run() {
            while (true) {
                try {
                    this.this0._stateLock.writeLock().lock();

                    for (int i = 0; i < 4; ++i) {
                        _swervePositions[i] = _modules[i].getPosition();
                    }

                    ChassisSpeeds speeds = _kinematics.toChassisSpeeds(getModuleStates());

                    double time = Timer.getFPGATimestamp();

                    Pose2d newPose = _poseEstimator.updateWithTime(this.this0._cachedState.timeStamp, latencyAdjustedYaw(), _swervePositions);

                    this.this0._cachedState.update(
                        newPose,
                        speeds,
                        time
                    );
                } finally {
                    this.this0._stateLock.writeLock().unlock();
                }
                Timer.delay(1.0 / 100.0);
            }
        }
    }
}
