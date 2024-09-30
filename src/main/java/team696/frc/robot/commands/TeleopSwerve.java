package team696.frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.lib.Util;
import team696.frc.lib.Swerve.SwerveConstants;
import team696.frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {
    protected static DoubleSupplier translation = ()->0;
    protected static DoubleSupplier strafe = ()->0;
    protected static DoubleSupplier rotation = ()->0;
    protected static double deadband = 1; // deadband for controller -> defaulted to 1 so you must config swerve
    protected static double rotationDeadband = 1;
    private static PIDController pidController = new PIDController(0.0056, 0.00, 0); 
    static {
        pidController.enableContinuousInput(-180, 180);
        pidController.setTolerance(1);
    }
    private static BooleanSupplier lockRotation; // should lock rotation -> usually a button

    private boolean fieldRelative; // should do fieldRelative controol
    private boolean openLoop; // should do openLoop control
    private Supplier<Rotation2d> goalRotation; // Rotation to lock to once lockRotation has been activated
    private DoubleSupplier multiplier = ()->1; // Multiplier to outputted Speed

    public static void config(DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, BooleanSupplier rotationLock, double deadBand) {
        strafe = x;
        translation = y;
        rotation = r;

        lockRotation = rotationLock;

        deadband = deadBand;
        rotationDeadband = Math.sqrt(2 * Math.pow(deadband, 2));
    }
    
    public TeleopSwerve(DoubleSupplier multiplier, Supplier<Rotation2d> goal, boolean fieldRelative, boolean openLoop) {
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        this.goalRotation = goal;

        this.multiplier = multiplier;

        addRequirements(Swerve.get());
    }

    public TeleopSwerve(DoubleSupplier multiplier, Supplier<Rotation2d> goal) {
        this(multiplier, goal, true, true);
    }


    public TeleopSwerve(DoubleSupplier multiplier) {
        this(multiplier, null, true, true);
    }

    public TeleopSwerve(Supplier<Rotation2d> goal) {
        this(()->1, goal, true, true);
    }

    public TeleopSwerve() {
        this(()->1, null, true, true);
    }

    @Override
    public void execute() {
        double yAxis = translation.getAsDouble();
        double xAxis = strafe.getAsDouble();
        double rAxis = rotation.getAsDouble();

        Rotation2d theta = new Rotation2d(yAxis, xAxis);
        double magnitude = Math.min(Math.sqrt((xAxis * xAxis) + (yAxis * yAxis)), 1);
        if (magnitude < deadband) magnitude = 0;

        if (lockRotation != null && lockRotation.getAsBoolean() && goalRotation.get() != null) { // Rotation Lock To Angle TODO: REWORK THIS PID
            double pid = pidController.calculate(Swerve.get().getPose().getRotation().getDegrees(), goalRotation.get().getDegrees());
            rAxis = Math.abs(pidController.getPositionError()) > 1 ? Math.abs(Math.pow(pid, 2)) * 1.1 * Math.signum(pid) + pid * 2.2 : 0;
        } else {
            rAxis = (Math.abs(rAxis) > rotationDeadband) ? Util.map(rAxis * rAxis, rotationDeadband, 1, 0, 1) * Math.signum(rAxis) : 0;
        }

        double rotation = rAxis * SwerveConstants.maxAngularVelocity;
        Translation2d translation = new Translation2d(Math.pow(magnitude, 2), theta).times(SwerveConstants.maxSpeed).times(multiplier.getAsDouble());

        Swerve.get().Drive(translation, rotation, fieldRelative, openLoop);
    }
}