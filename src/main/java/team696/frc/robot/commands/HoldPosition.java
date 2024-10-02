package team696.frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Swerve;

public class HoldPosition extends Command{
 
    Pose2d desired;

    static PIDController x = new PIDController(2., 0, 0);
    static PIDController y = new PIDController(2., 0, 0);
    static PIDController r = new PIDController(0.1, 0, 0);

    static {
        r.enableContinuousInput(-180, 180);
    }

    public HoldPosition(Pose2d desired) {
        this.desired = desired;
    }

    @Override
    public void execute() {
        if (Swerve.get().getPose().getTranslation().getDistance(desired.getTranslation()) > 1) {
            Swerve.get().Drive(new Translation2d(), 0,true,true);
            return;
        }

        double drivex = x.calculate(Swerve.get().getPose().getX(), desired.getX());
        double drivey = y.calculate(Swerve.get().getPose().getY(), desired.getY());
        double driver = r.calculate(Swerve.get().getPose().getRotation().getDegrees(), desired.getRotation().getDegrees());


        Swerve.get().Drive(new Translation2d(drivex, drivey), driver, true, true);
    }
}
