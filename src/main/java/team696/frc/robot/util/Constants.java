package team696.frc.robot.util;

import java.util.Map;
import java.util.TreeMap;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class Constants {
	public static final Configs configs = new Configs();

	public static final double epsilon = 0.0000001; // just a really small number

	public static final boolean DEBUG = true;

	public static final String canivoreName = "vore";

	public static final double deadBand = 0.03;
	public static final class Field {
		public static final Field2d sim = new Field2d();
        public static final class RED {
		    public static final Translation2d Speaker = new Translation2d(16.71, 5.54);
            public static final Pose2d Amp = new Pose2d(12, 8, new Rotation2d(Math.PI/2));
            public static final Pose2d Source = new Pose2d(1, 0.5, Rotation2d.fromDegrees(-135));
        }
        public static final class BLUE {
            public static final Translation2d Speaker = new Translation2d(-0.11, 5.54);
            public static final Pose2d Amp = new Pose2d(4, 8, new Rotation2d(Math.PI/2));
            public static final Pose2d Source = new Pose2d(15.15, 1.5, Rotation2d.fromDegrees(135)); 
        }
	}
	public static class Motors {
        public static final class Falcon {
			@AutoLogOutput
            public static final double stallCurA = 257;
            public static final double stallTorqueNm = 4.69;
            public static final double freeSpinRPM = 6380;
            public static final double freeSpinA = 1.5;
        }
        public static final class Kraken {
            public static final double stallCurA = 366;
            public static final double stallTorqueNm = 7.09;
            public static final double freeSpinRPM = 6000;
            public static final double freeSpinA = 1.5;
        }
	}
	public static class swerve {
		public static final double drivekS = (0.667 / 12); 
		public static final double drivekV = (2.44 / 12);
		public static final double drivekA = (0.27 / 12);

		public static final double driveGearRatio = (5.36 / 1.0); // L3
		public static final double angleGearRatio = (150.0/7.0 / 1.0); 

		public static final double massKgs = Units.lbsToKilograms(70);

		public static final double wheelX = Units.inchesToMeters(13.0);
		public static final double wheelY = Units.inchesToMeters(13.0);
		public static final double wheelDiameter = Units.inchesToMeters(3.89);
		public static final double wheelCircumference = wheelDiameter * Math.PI;

		public static final double theoreticalMaxSpeed = Motors.Kraken.freeSpinRPM / 60 / driveGearRatio * wheelCircumference; // 5.13 mps way more resonable
		public static final double theoreticalMaxAcceleration = (4 * Motors.Kraken.stallTorqueNm * driveGearRatio) / (massKgs * wheelDiameter / 2);  // 66 mps^2 wtf
		public static final double maxSpeed = theoreticalMaxSpeed * 0.9; //MPS
		public static final double maxAngularVelocity = 8; //MPS^2

		private static final SwerveModule frontLeft = new SwerveModule(0, Constants.configs.swerve.Mod0);
		private static final SwerveModule frontRight = new SwerveModule(1, Constants.configs.swerve.Mod1);
		private static final SwerveModule backLeft = new SwerveModule(2, Constants.configs.swerve.Mod2);
		private static final SwerveModule backRight = new SwerveModule(3, Constants.configs.swerve.Mod3);
		public static final SwerveModule[] modules = { frontLeft, frontRight, backLeft, backRight };

		public static final Translation2d[] modPositions = {
			new Translation2d(wheelX / 2.0, wheelY / 2.0), // FL
			new Translation2d(wheelX / 2.0, -wheelY / 2.0), // FR
			new Translation2d(-wheelX / 2.0, wheelY / 2.0), // BL
			new Translation2d(-wheelX / 2.0, -wheelY / 2.0) // BR  
		};

		public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(modPositions);
	}

	public static class shooter {
		public static final double kG = 0.042;

		public static class state {
			public double angle;

			public double speed_l;
			public double speed_r;

			public state(double a, double l, double r) {
				angle = a;
				speed_l = l;
				speed_r = r;
			}
		}

		public static final TreeMap<Double, state> distToState = new TreeMap<Double, state>(){{
			put(1.5, new state(4.2, 3500, 3500));
			put(2.0, new state(3.5, 3500, 3500));
			put(2.5, new state(2.9, 3500, 3500));
			put(3.0, new state(2.2, 3500, 3500));
			put(3.5, new state(1.7, 3500, 3500));
			put(4.0, new state(1.5, 3500, 3500));
			put(5.0, new state(1.2, 3500, 3500));
			put(12., new state(1.0, 3500, 3500));
		}};

		public static final state adjustedState(double dist) {
			dist = Util.clamp(dist, Constants.shooter.distToState.firstKey() + epsilon, Constants.shooter.distToState.lastKey() - epsilon);
        	Map.Entry<Double, state> lower = Constants.shooter.distToState.floorEntry(dist);
        	Map.Entry<Double, state> higher = Constants.shooter.distToState.ceilingEntry(dist);

        	return new state(
				Util.lerp((dist - lower.getKey())/(higher.getKey() - lower.getKey()), lower.getValue().angle, higher.getValue().angle), 
				higher.getValue().speed_l, 
				higher.getValue().speed_r);
		}
	}

	public static class Robot {
		public enum Robots {
			SIM,
			UNKNOWN,
			COMP,
			BETA
		}

		public static Robots detected = Robots.UNKNOWN;
		public static final byte[] COMP_MAC = new byte[]{ (byte) 0x00, (byte) 0x80, (byte) 0x2F, (byte) 0x38, (byte) 0x5F, (byte) 0x75 };
		public static final byte[] BETA_MAC = new byte[]{ (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x35, (byte) 0xb8, (byte) 0xca };
	}
}
