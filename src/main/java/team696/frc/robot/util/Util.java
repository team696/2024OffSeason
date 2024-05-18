package team696.frc.robot.util;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

import team696.frc.lib.Log.PLog;
import team696.frc.robot.Robot;

public class Util {
    public static double lerp(double t, double min, double max) {
        return (max - min) * t + min;
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(Math.min(max, val), min);
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {
            PLog.fatalException("Sleep", "Failed To Sleep", e);
        }
    }

    /**
     * @param wheelRPS Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double RPSToMPS(double wheelRPS, double circumference){
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelMPS Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference){
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
    }

    /**
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    public static double rotationsToMeters(double wheelRotations, double circumference){
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }

    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference){
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }

    public static List<byte[]> getMacAddresses() throws IOException {
		List<byte[]> macAddresses = new ArrayList<>();

		Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();
		NetworkInterface networkInterface;
		while (networkInterfaces.hasMoreElements()) {
			networkInterface = networkInterfaces.nextElement();

			byte[] address = networkInterface.getHardwareAddress();
			if (address == null) {
				continue;
			}

			macAddresses.add(address);
		}
		return macAddresses;
	}

    public static String macToString(byte[] address) {
		StringBuilder builder = new StringBuilder();
		for (int i = 0; i < address.length; i++) {
			if (i != 0) {
				builder.append(':');
			}
			builder.append(String.format("%02X", address[i]));
		}
		return builder.toString();
	}

    public static void setRobotType () {
        if (Robot.isSimulation()) {
            Constants.Robot.detected = Constants.Robot.Robots.SIM;
            PLog.info("Robot", "Simulation Detected");
            return;
        }

        List<byte[]> macAddresses;
		try {
			macAddresses = Util.getMacAddresses();
		} catch (IOException e) {
            PLog.fatalException("Robot", "Mac Address Attempt Unsuccessful", e);
			macAddresses = List.of();
		}

		for (byte[] macAddress : macAddresses) {
			if (Arrays.compare(Constants.Robot.COMP_MAC, macAddress) == 0) {
				Constants.Robot.detected = Constants.Robot.Robots.COMP;
                PLog.info("Robot", "Comp Bot Detected");
				break;
			} else if (Arrays.compare(Constants.Robot.BETA_MAC, macAddress) == 0) {
				Constants.Robot.detected = Constants.Robot.Robots.BETA;
                PLog.info("Robot", "Beta Bot Detected");
				break;
			}
		}

		if (Constants.Robot.detected == Constants.Robot.Robots.UNKNOWN) {
            PLog.info("Robot", "Unknown MAC address!");
            for (byte[] macAddress : macAddresses) {
                PLog.info("    ", Util.macToString(macAddress));
            }
		}
    }
}
