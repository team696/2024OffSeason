package team696.frc.robot.util;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import team696.frc.lib.TalonFactory;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;

    private TalonFactory mAngleMotor;
    private TalonFactory mDriveMotor;
    private CANcoder angleEncoder;

    private double m_lastAngle;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.swerve.drivekS, Constants.swerve.drivekV, Constants.swerve.drivekA);
    
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.CANcoderOffset; 
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.CANcoderId);
        angleEncoder.getConfigurator().apply(Constants.CONFIGS.swerve_CANCoder);

        /* Angle Motor Config */
        mAngleMotor = new TalonFactory(moduleConstants.SteerMotorId, Constants.CONFIGS.swerve_Angle, "Swerve Angle " + moduleNumber);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFactory(moduleConstants.DriveMotorId, Constants.CONFIGS.swerve_Drive, "Swerve Drive " + moduleNumber);

        m_lastAngle = getState().angle.getRotations();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        double angle = ((Math.abs(desiredState.speedMetersPerSecond) <= (Constants.swerve.maxSpeed * 0.01)) ? m_lastAngle : desiredState.angle.getRotations());
        mAngleMotor.setControl(anglePosition.withPosition(angle));
        setSpeed(desiredState, isOpenLoop);
        m_lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        double ratio = Math.cos(desiredState.angle.getRadians() - getState().angle.getRadians()); 
        if(isOpenLoop){
            mDriveMotor.VoltageOut(desiredState.speedMetersPerSecond / Constants.swerve.maxSpeed * ratio);
        } else {
            driveVelocity.Velocity = Util.MPSToRPS(desiredState.speedMetersPerSecond * ratio, Constants.swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANCoderAngle(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANCoderAngle().getRotations() - angleOffset;
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Util.RPSToMPS(mDriveMotor.getVelocity(), Constants.swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Util.rotationsToMeters(mDriveMotor.getPosition(), Constants.swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition())
        );
    }
}