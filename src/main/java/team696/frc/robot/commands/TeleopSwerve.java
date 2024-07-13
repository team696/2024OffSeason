package team696.frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Swerve;
import team696.frc.robot.util.Constants;
import team696.frc.robot.util.Util;

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
    private Supplier<Translation2d> goalPose;
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

    public TeleopSwerve(double multiplier, DoubleSupplier goal, boolean fieldRelative, boolean openLoop, Supplier<Translation2d> goalPose) {
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        this.goalRotation = goal;
        this.goalPose = goalPose;

        this.multiplier = multiplier;

        addRequirements(Swerve.get());
    }

    public TeleopSwerve(double multiplier) {
        this(multiplier, ()->0, true, true, null);
    }

    public TeleopSwerve(DoubleSupplier goal, Supplier<Translation2d> goalPose) {
        this(1, goal, true, true, goalPose);
    }

    public TeleopSwerve(Supplier<Translation2d> goal) {
        this(1, ()->0, true, true, goal);
    }

    public TeleopSwerve(DoubleSupplier goal) {
        this(1, goal, true, true, null);
    }

    public TeleopSwerve() {
        this(1, ()->0, true, true, null);
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

        if (goalPose != null) { // Aim Assist
            Translation2d diff = Swerve.get().getPose().getTranslation().minus(goalPose.get());

            Rotation2d angleToGoal = Rotation2d.fromRadians(Math.atan2(diff.getY(), diff.getX()));

            Rotation2d rotDiff = theta.minus(angleToGoal);

            theta = theta.plus(rotDiff.times( Math.min(1, 1/diff.getNorm() / 3 ) ));
        }

        if (lockRotation != null && lockRotation.getAsBoolean()) { // Rotation Lock To Angle TODO: REWORK THIS PID
            double pid = pidController.calculate(Swerve.get().getPose().getRotation().getDegrees(), goalRotation.getAsDouble());
            if (Math.abs(pidController.getPositionError()) > 1)
                rAxis = Math.abs(Math.pow(pid, 2)) * 1.1 * Math.signum(pid) + pid * 2.2;
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