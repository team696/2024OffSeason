package team696.frc.robot;

import java.util.Map;
import java.util.TreeMap;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import team696.frc.lib.Util;

public final class Constants {
	public static final Configs configs = new Configs();

	public static final double epsilon = 0.0000001; // just a really small number

	public static final boolean DEBUG = true;

	public static final String canivoreName = "vore";

	public static final double deadBand = 0.05;
	public static final class Field {
        public static final class RED {
		    public static final Translation2d Speaker = new Translation2d(16.57, 5.54);
            public static final Pose2d Amp = new Pose2d(14.7, 7.8, new Rotation2d(Math.PI/2));
            public static final Pose2d Source = new Pose2d(1, 0.5, Rotation2d.fromDegrees(-135));
			public static final Translation2d Corner = new Translation2d(14.57, 7.);

        }
        public static final class BLUE {
            public static final Translation2d Speaker = new Translation2d(-0.04, 5.54);
            public static final Pose2d Amp = new Pose2d(1.7, 7.8, new Rotation2d(Math.PI/2));
            public static final Pose2d Source = new Pose2d(15.15, 1.5, Rotation2d.fromDegrees(135)); 
			public static final Translation2d Corner = new Translation2d(2., 7.);

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
	
	public static class shooter {
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

		public static final double rollerSpeed = 4100;
		public static final double rollerSpeedA = 3700;

		public static final double globalOffset = -0.05;

		public static final TreeMap<Double, state> distToState = new TreeMap<Double, state>(){{
			put(1.5, new state(4.30 + globalOffset, 3200, 3200));
			put(2.0, new state(3.30 + globalOffset, 3200, 3200));
			put(2.5, new state(2.40 + globalOffset, 3200, 3200));
			put(3.0, new state(1.60 + globalOffset, rollerSpeed, rollerSpeedA));
			put(3.5, new state(1.30 + globalOffset, rollerSpeed, rollerSpeedA));
			put(4.0, new state(0.90 + globalOffset, rollerSpeed, rollerSpeedA));
			put(4.5, new state(0.65 + globalOffset, rollerSpeed, rollerSpeedA));
			put(5.0, new state(0.40 + globalOffset, rollerSpeed, rollerSpeedA));
			put(5.5, new state(0.30 + globalOffset, rollerSpeed, rollerSpeedA));
			put(6.0, new state(0.30 + globalOffset, rollerSpeed, rollerSpeedA));

			put(12., new state(0.4, rollerSpeed, rollerSpeedA));
		}};

		public static final TreeMap<Double, state> Pass = new TreeMap<Double, state>(){{
			put(1.5, new state(0.0, 1800, 1800));
			put(4.0, new state(0.0, 1800, 1800));
			put(5.0, new state(0.0, 2500, 2500));
			put(6.0, new state(3.3, 3200, 3200));
			put(8.0, new state(3.3, 3200, 3200));
			put(12., new state(3.3, 3800, 3800));
		}};


		public static final state adjustedState(double dist) {
			dist = Util.clamp(dist, Constants.shooter.distToState.firstKey() + epsilon, Constants.shooter.distToState.lastKey() - epsilon);
        	Map.Entry<Double, state> lower = Constants.shooter.distToState.floorEntry(dist);
        	Map.Entry<Double, state> higher = Constants.shooter.distToState.ceilingEntry(dist);

			double t = (dist - lower.getKey()) / (higher.getKey() - lower.getKey());

        	return new state(
				Util.lerp(t, lower.getValue().angle, higher.getValue().angle), 
				Util.lerp(t, lower.getValue().speed_l, higher.getValue().speed_l), 
				Util.lerp(t, lower.getValue().speed_r, higher.getValue().speed_r));
		}

		public static final state adjustedPassState(double dist) {
			dist = Util.clamp(dist, Constants.shooter.Pass.firstKey() + epsilon, Constants.shooter.Pass.lastKey() - epsilon);
        	Map.Entry<Double, state> lower = Constants.shooter.Pass.floorEntry(dist);
        	Map.Entry<Double, state> higher = Constants.shooter.Pass.ceilingEntry(dist);

			double t = (dist - lower.getKey()) / (higher.getKey() - lower.getKey());

        	return new state(
				Util.lerp(t, lower.getValue().angle, higher.getValue().angle), 
				Util.lerp(t, lower.getValue().speed_l, higher.getValue().speed_l), 
				Util.lerp(t, lower.getValue().speed_r, higher.getValue().speed_r));
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
