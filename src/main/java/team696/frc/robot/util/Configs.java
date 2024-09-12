package team696.frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class Configs { 
    public final class Swerve {
        public TalonFXConfiguration angle;
        public TalonFXConfiguration drive;
        public CANcoderConfiguration canCoder;
        public Pigeon2Configuration pigeon;
        public SwerveModuleConstants Mod0;
        public SwerveModuleConstants Mod1;
        public SwerveModuleConstants Mod2;
        public SwerveModuleConstants Mod3;
    }

    public final class Hood {
        public TalonFXConfiguration left;
        public TalonFXConfiguration right;
    }

    public final class Shooter {
        public TalonFXConfiguration left;
        public TalonFXConfiguration right;
        public TalonFXConfiguration serializer;
    }

    public final class Intake {
        public TalonFXConfiguration serializer;
    }

    public Swerve swerve = new Swerve();
    public Hood hood = new Hood();
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();

    public Configs() {
        swerve.angle = new TalonFXConfiguration();
        swerve.drive = new TalonFXConfiguration();
        swerve.canCoder = new CANcoderConfiguration();
        swerve.pigeon = new Pigeon2Configuration();
        swerve.Mod0 = new SwerveModuleConstants();
        swerve.Mod1 = new SwerveModuleConstants();
        swerve.Mod2 = new SwerveModuleConstants();
        swerve.Mod3 = new SwerveModuleConstants();

        hood.left = new TalonFXConfiguration();
        hood.right = new TalonFXConfiguration();
    
        shooter.left = new TalonFXConfiguration();
        shooter.right = new TalonFXConfiguration();
        shooter.serializer = new TalonFXConfiguration();

        intake.serializer = new TalonFXConfiguration();
    
        /** Swerve CANCoder Configuration */
        swerve.canCoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        swerve.canCoder.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        /** Swerve Angle Motor Configuration */
        swerve.angle.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerve.angle.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        swerve.angle.Feedback.SensorToMechanismRatio = Constants.swerve.angleGearRatio;
        swerve.angle.ClosedLoopGeneral.ContinuousWrap = true;
        swerve.angle.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerve.angle.CurrentLimits.SupplyCurrentLimit = 25;
        swerve.angle.CurrentLimits.SupplyCurrentThreshold = 40;
        swerve.angle.CurrentLimits.SupplyTimeThreshold = 0.1;
        swerve.angle.CurrentLimits.StatorCurrentLimitEnable = true;
        swerve.angle.CurrentLimits.StatorCurrentLimit = 40;
        swerve.angle.Slot0.kP = 128.0;
        swerve.angle.Slot0.kI = 0.0;
        swerve.angle.Slot0.kD = 0.0;

        swerve.angle.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
        swerve.angle.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

        swerve.angle.Voltage.PeakForwardVoltage = 12.;
        swerve.angle.Voltage.PeakReverseVoltage = -12.;

        /** Swerve Drive Motor Configuration */
        swerve.drive.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerve.drive.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        swerve.drive.Feedback.SensorToMechanismRatio = Constants.swerve.driveGearRatio;
        swerve.drive.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerve.drive.CurrentLimits.SupplyCurrentLimit = 25;
        swerve.drive.CurrentLimits.SupplyCurrentThreshold = 60;
        swerve.drive.CurrentLimits.SupplyTimeThreshold = 0.2;
        swerve.drive.CurrentLimits.StatorCurrentLimitEnable = true;
        swerve.drive.CurrentLimits.StatorCurrentLimit = 60;

        swerve.drive.Voltage.PeakForwardVoltage = 12.;
        swerve.drive.Voltage.PeakReverseVoltage = -12.;

        swerve.drive.Slot0.kP = 2.;
        swerve.drive.Slot0.kI = 0.0;
        swerve.drive.Slot0.kD = 0.0;
        swerve.drive.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        swerve.drive.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
        swerve.drive.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;
        swerve.drive.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

        /** Individual Swerve Module Configurations -> frontLeft, frontRight, backLeft, backRight */ 
        swerve.Mod0.CANcoderId = 0;  
        swerve.Mod0.DriveMotorId = 3; 
        swerve.Mod0.SteerMotorId = 6;
        swerve.Mod0.CANcoderOffset = -0.307;
 
        swerve.Mod1.CANcoderId = 3; 
        swerve.Mod1.DriveMotorId = 4;
        swerve.Mod1.SteerMotorId = 7;
        swerve.Mod1.CANcoderOffset = 0.268;

        swerve.Mod2.CANcoderId = 2; 
        swerve.Mod2.DriveMotorId = 2;
        swerve.Mod2.SteerMotorId = 1;
        swerve.Mod2.CANcoderOffset = 0.371;

        swerve.Mod3.CANcoderId = 1; 
        swerve.Mod3.DriveMotorId = 5;
        swerve.Mod3.SteerMotorId = 0;
        swerve.Mod3.CANcoderOffset = 0.357;

        /** Pigeon Configuration */ 
        swerve.pigeon.MountPose.MountPoseYaw = -177;

        hood.left.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hood.left.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hood.left.CurrentLimits.StatorCurrentLimitEnable = true;
        hood.left.CurrentLimits.StatorCurrentLimit = 80;
        hood.left.Slot0.kP = 4.; 
        hood.left.Slot0.kS = 0.6;
        hood.left.Slot0.kV = 0;
        hood.left.Slot0.kA = 0;
        hood.left.MotionMagic.MotionMagicCruiseVelocity = 20;
        hood.left.MotionMagic.MotionMagicAcceleration = 50;

        hood.right.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hood.right.MotorOutput.NeutralMode = hood.left.MotorOutput.NeutralMode;
        hood.right.Slot0.kP = hood.left.Slot0.kP;
        hood.right.CurrentLimits.StatorCurrentLimitEnable = true;
        hood.right.CurrentLimits.StatorCurrentLimit = 80;
        hood.right.Slot0.kP = hood.left.Slot0.kP; 
        hood.right.Slot0.kS = hood.left.Slot0.kS;
        hood.right.Slot0.kV = hood.left.Slot0.kV;
        hood.right.Slot0.kA = hood.left.Slot0.kA;
        hood.right.MotionMagic.MotionMagicCruiseVelocity = hood.left.MotionMagic.MotionMagicCruiseVelocity;
        hood.right.MotionMagic.MotionMagicAcceleration = hood.left.MotionMagic.MotionMagicAcceleration;

        shooter.left.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooter.left.Slot0.kP = 0.35;
        shooter.left.Slot0.kV = 0.14;
        shooter.left.Slot0.kS = 0.14;
        shooter.left.CurrentLimits.StatorCurrentLimitEnable = true;
        shooter.left.CurrentLimits.StatorCurrentLimit = 80;

        shooter.left.Voltage.PeakForwardVoltage = 12.;
        shooter.left.Voltage.PeakReverseVoltage = -12.;

        shooter.right.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooter.right.Slot0.kP = shooter.left.Slot0.kP;
        shooter.right.Slot0.kV = shooter.left.Slot0.kV;
        shooter.right.Slot0.kS = shooter.left.Slot0.kS;
        shooter.right.CurrentLimits.StatorCurrentLimitEnable = true;
        shooter.right.CurrentLimits.StatorCurrentLimit = 80;

        shooter.right.Voltage.PeakForwardVoltage = 12.;
        shooter.right.Voltage.PeakReverseVoltage = -12.;

        intake.serializer.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intake.serializer.CurrentLimits.StatorCurrentLimitEnable = false;
        intake.serializer.CurrentLimits.StatorCurrentLimit = 80;
        intake.serializer.Voltage.PeakForwardVoltage = 12.;
        intake.serializer.Voltage.PeakReverseVoltage = -12.;
        shooter.serializer.Slot0.kP = 12.;
    }
}