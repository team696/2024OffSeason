package team696.frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import team696.frc.lib.Datatypes.InterpolatingTable;
import team696.frc.robot.subsystems.Shooter;

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
            public static final Pose2d Source = new Pose2d(15.15, 1.5, Rotation2d.fromDegrees(-45)); 
			public static final Translation2d Corner = new Translation2d(2., 7.);

        }
	}
	
	public static class shooter {
		public static final double rollerSpeed = 4100;
		public static final double rollerSpeedA = 3700;

		public static final double globalOffset = 0.45;

		public static final InterpolatingTable<Shooter.state> ShooterTable = new InterpolatingTable<>(
			Map.entry(1.5, new Shooter.state(4.20 + globalOffset, 3600, 3300)),
			Map.entry(2.0, new Shooter.state(3.30 + globalOffset, 3600, 3300)),
			Map.entry(2.5, new Shooter.state(2.45 + globalOffset, 3600, 3300)),
			Map.entry(3.0, new Shooter.state(1.65 + globalOffset, rollerSpeed, rollerSpeedA)),
			Map.entry(3.5, new Shooter.state(1.35 + globalOffset, rollerSpeed, rollerSpeedA)),
			Map.entry(4.0, new Shooter.state(1.15 + globalOffset, rollerSpeed, rollerSpeedA)),
			Map.entry(4.5, new Shooter.state(1.00 + globalOffset, rollerSpeed, rollerSpeedA)),
			Map.entry(5.0, new Shooter.state(0.70 + globalOffset, rollerSpeed, rollerSpeedA)),
			Map.entry(5.5, new Shooter.state(0.55 + globalOffset, rollerSpeed, rollerSpeedA)),
			Map.entry(6.0, new Shooter.state(0.55 + globalOffset, rollerSpeed, rollerSpeedA)),
			Map.entry(12., new Shooter.state(0.4, rollerSpeed, rollerSpeedA))
		);
		public static final InterpolatingTable<Shooter.state> PassTable = new InterpolatingTable<>(
			Map.entry(1.5, new Shooter.state(0.0, 1800, 1800)),
			Map.entry(4.0, new Shooter.state(0.0, 1800, 1800)),
			Map.entry(5.0, new Shooter.state(0.0, 2500, 2500)),
			Map.entry(6.0, new Shooter.state(3.3, 3200, 3200)),
			Map.entry(8.0, new Shooter.state(3.3, 3200, 3200)),
			Map.entry(12., new Shooter.state(3.3, 3800, 3800))
		);
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
