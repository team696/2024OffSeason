package frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class Configs {

    public TalonFXConfiguration swerve_Angle = new TalonFXConfiguration();
    public TalonFXConfiguration swerve_Drive = new TalonFXConfiguration();
    public CANcoderConfiguration swerve_CANCoder = new CANcoderConfiguration();
    public Pigeon2Configuration swerve_Pigeon = new Pigeon2Configuration();
    public SwerveModuleConstants Mod0 = new SwerveModuleConstants();
    public SwerveModuleConstants Mod1 = new SwerveModuleConstants();
    public SwerveModuleConstants Mod2 = new SwerveModuleConstants();
    public SwerveModuleConstants Mod3 = new SwerveModuleConstants();
    
    public TalonFXConfiguration shooter_Top = new TalonFXConfiguration();
    public TalonFXConfiguration shooter_Bottom = new TalonFXConfiguration();
    public TalonFXConfiguration shooter_Angle = new TalonFXConfiguration();
    public TalonFXConfiguration shooter_Serializer = new TalonFXConfiguration();

    public TalonFXConfiguration intake_Angle = new TalonFXConfiguration();
    public TalonFXConfiguration intake_Serializer = new TalonFXConfiguration();
    public TalonFXConfiguration intake_Rollers = new TalonFXConfiguration();

    public TalonFXConfiguration climber_Master = new TalonFXConfiguration();
    public TalonFXConfiguration climber_Follower = new TalonFXConfiguration();

    public Configs() {
        /** Swerve CANCoder Configuration */
        swerve_CANCoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        swerve_CANCoder.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        /** Swerve Angle Motor Configuration */
        swerve_Angle.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerve_Angle.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        swerve_Angle.Feedback.SensorToMechanismRatio = Constants.swerve.angleGearRatio;
        swerve_Angle.ClosedLoopGeneral.ContinuousWrap = true;
        swerve_Angle.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerve_Angle.CurrentLimits.SupplyCurrentLimit = 25;
        swerve_Angle.CurrentLimits.SupplyCurrentThreshold = 40;
        swerve_Angle.CurrentLimits.SupplyTimeThreshold = 0.1;
        swerve_Angle.Slot0.kP = 128.0;
        swerve_Angle.Slot0.kI = 0.0;
        swerve_Angle.Slot0.kD = 0.0;

        swerve_Angle.Voltage.PeakForwardVoltage = 12.;
        swerve_Angle.Voltage.PeakReverseVoltage = -12.;

        /** Swerve Drive Motor Configuration */
        swerve_Drive.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerve_Drive.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        swerve_Drive.Feedback.SensorToMechanismRatio = Constants.swerve.driveGearRatio;
        swerve_Drive.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerve_Drive.CurrentLimits.SupplyCurrentLimit = 25;
        swerve_Drive.CurrentLimits.SupplyCurrentThreshold = 60;
        swerve_Drive.CurrentLimits.SupplyTimeThreshold = 0.2;
        swerve_Drive.CurrentLimits.StatorCurrentLimitEnable = true;
        swerve_Drive.CurrentLimits.StatorCurrentLimit = 60;

        swerve_Drive.Voltage.PeakForwardVoltage = 12.;
        swerve_Drive.Voltage.PeakReverseVoltage = -12.;

        swerve_Drive.Slot0.kP = 2.;
        swerve_Drive.Slot0.kI = 0.0;
        swerve_Drive.Slot0.kD = 0.0;
        swerve_Drive.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        swerve_Drive.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
        swerve_Drive.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;
        swerve_Drive.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

        /** Individual Swerve Module Configurations -> frontLeft, frontRight, backLeft, backRight */ 
        Mod0.CANcoderId = 0;  
        Mod0.DriveMotorId = 3; 
        Mod0.SteerMotorId = 6;
        Mod0.CANcoderOffset = -0.307;
 
        Mod1.CANcoderId = 3; 
        Mod1.DriveMotorId = 4;
        Mod1.SteerMotorId = 7;
        Mod1.CANcoderOffset = 0.268;

        Mod2.CANcoderId = 2; 
        Mod2.DriveMotorId = 2;
        Mod2.SteerMotorId = 1;
        Mod2.CANcoderOffset = 0.371;

        Mod3.CANcoderId = 1; 
        Mod3.DriveMotorId = 5;
        Mod3.SteerMotorId = 0;
        Mod3.CANcoderOffset = 0.357;

        /** Pigeon Configuration */ 
        swerve_Pigeon.MountPose.MountPoseYaw = -177;
    }
}