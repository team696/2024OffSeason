package team696.frc.lib.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team696.frc.lib.Util;
import team696.frc.lib.HardwareDevices.CANCoderFactory;
import team696.frc.lib.HardwareDevices.TalonFactory;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

public class SwerveModule implements Sendable{
    public int moduleNumber;
    private double _angleOffset;
    
    private TalonFactory _angleMotor;
    private TalonFactory _driveMotor;
    private CANCoderFactory _encoder;

    private double _lastAngle;

    private final SimpleMotorFeedforward _driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.drivekS, SwerveConstants.drivekV, SwerveConstants.drivekA);
    
    private final VelocityVoltage _driveVelocity = new VelocityVoltage(0);

    private final PositionVoltage _anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this._angleOffset = moduleConstants.CANcoderOffset; 
        /* Angle Encoder Config */
        _encoder = new CANCoderFactory(moduleConstants.CANcoderId, SwerveConstants.canBus, SwerveConfigs.canCoder, "Swerve Encoder " + moduleNumber);

        /* Angle Motor Config */
        _angleMotor = new TalonFactory(moduleConstants.SteerMotorId, SwerveConstants.canBus, SwerveConfigs.angle, "Swerve Angle " + moduleNumber);
        resetToAbsolute();

        /* Drive Motor Config */
        _driveMotor = new TalonFactory(moduleConstants.DriveMotorId, SwerveConstants.canBus, SwerveConfigs.drive, "Swerve Drive " + moduleNumber);

        _encoder.get().getAbsolutePosition().setUpdateFrequency(100);
        _angleMotor.get().getPosition().setUpdateFrequency(100);
        _angleMotor.get().getVelocity().setUpdateFrequency(100);
        _driveMotor.get().getPosition().setUpdateFrequency(100);
        _driveMotor.get().getVelocity().setUpdateFrequency(100);

        ParentDevice.optimizeBusUtilizationForAll(_encoder.get(), _angleMotor.get(), _driveMotor.get());

        _lastAngle = getState().angle.getRotations();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        double angle = ((Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01)) ? _lastAngle : desiredState.angle.getRotations());
        _angleMotor.setControl(_anglePosition.withPosition(angle));
        setSpeed(desiredState, isOpenLoop);
        _lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        double ratio = Math.cos(desiredState.angle.getRadians() - getState().angle.getRadians()); 
        if(isOpenLoop){
            _driveMotor.VoltageOut(desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed * ratio);
        } else {
            _driveVelocity.Velocity = Util.MPSToRPS(desiredState.speedMetersPerSecond * ratio, SwerveConstants.wheelCircumference);
            _driveVelocity.FeedForward = _driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            _driveMotor.setControl(_driveVelocity);
        }
    }

    public Rotation2d getCANCoderAngle(){
        return _encoder.getPosition();
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANCoderAngle().getRotations() - _angleOffset;
        _angleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Util.RPSToMPS(_driveMotor.getVelocity(), SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(getAngleMotorPosition())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Util.rotationsToMeters(getDriveMotorPosition(), SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(getAngleMotorPosition())
        );
    }

    public double getDriveMotorPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(_driveMotor.get().getPosition(), _driveMotor.get().getVelocity());
    }

    public double getAngleMotorPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(_angleMotor.get().getPosition(), _angleMotor.get().getVelocity());
    }

    public void putData() {
        SmartDashboard.putData("Swerve/Mod " + this.moduleNumber, this);
    }

    @Override 
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Motor Angle", this._angleMotor::getPosition, null);
        builder.addDoubleProperty("Velocity", this._driveMotor::getVelocity, null);
        builder.addDoubleProperty("Encoder Angle", ()->this.getCANCoderAngle().getRotations(), null);
    }
}