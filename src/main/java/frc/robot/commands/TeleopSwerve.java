package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import frc.robot.util.Util;

public class TeleopSwerve extends Command {
    private static DoubleSupplier translation = ()->0;
    private static DoubleSupplier strafe = ()->0;
    private static DoubleSupplier rotation = ()->0;
    private static double deadband = 1;
    private static PIDController pidController;
    private static BooleanSupplier lockRotation;

    private boolean fieldRelative;
    private boolean openLoop;
    private DoubleSupplier goalRotation = ()->0;
    private double multiplier = 1;

    public static void config(DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, BooleanSupplier rotationLock, double deadBand) {
        strafe = x;
        translation = y;
        rotation = r;

        lockRotation = rotationLock;

        deadband = deadBand;

        pidController = new PIDController(0.0056, 0.00, 0);
        pidController.enableContinuousInput(-180, 180);
    }

    public TeleopSwerve(double multiplier, DoubleSupplier goal, boolean fieldRelative, boolean openLoop) {
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        this.goalRotation = goal;

        this.multiplier = multiplier;

        addRequirements(Swerve.get());
    }

    public TeleopSwerve(double multiplier) {
        this(multiplier, ()->0, true, true);
    }

    public TeleopSwerve() {
        this(1, ()->0, true, true);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double yAxis = translation.getAsDouble();
        double xAxis = strafe.getAsDouble();
        double rAxis = rotation.getAsDouble();

        Rotation2d theta = new Rotation2d(yAxis, xAxis);
        double magnitude = Math.min(Math.sqrt((xAxis * xAxis) + (yAxis * yAxis)),1);
        if (magnitude < deadband) magnitude = 0;

        if (lockRotation != null && lockRotation.getAsBoolean()) { // Rotation Lock To Angle TODO: REWORK THIS PID
            double pid = pidController.calculate(Swerve.get().getPose().getRotation().getDegrees(), goalRotation.getAsDouble());
            if (Math.abs(pidController.getPositionError()) > 1)
                rAxis = Math.abs(Math.pow(pid, 2)) * 0.7 * Math.signum(pid) + pid * 2.2;
            else    
                rAxis = 0; 
        } else {
            if (Math.abs(rAxis) > deadband) {
                if (rAxis > 0)
                    rAxis = Util.map(rAxis * rAxis, deadband, 1, 0, 1);
                else 
                    rAxis = Util.map(rAxis * rAxis * -1, -deadband, -1, 0, -1);
            } else {
                rAxis = 0;
            }
        }

        double rotation = rAxis * Constants.swerve.maxAngularVelocity;
        Translation2d translation = new Translation2d(Math.pow(magnitude, 2), theta).times(Constants.swerve.maxSpeed).times(multiplier);

        Swerve.get().Drive(translation, rotation, fieldRelative, openLoop);
    }
}