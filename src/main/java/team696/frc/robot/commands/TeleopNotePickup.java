// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import team696.frc.lib.Swerve.SwerveConstants;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class TeleopNotePickup extends TeleopSwerve {

    private PIDController rotationController = new PIDController(0.014, 0, 0);

   public TeleopNotePickup(Supplier<Rotation2d> goal) {
        super(()->1, goal, true, true);
    }


    @Override
    public void execute() {
        if (Swerve.get().getAngleForNote() == null) {
            super.execute();
            return;
        }

        double yAxis = translation.getAsDouble();
        double xAxis = strafe.getAsDouble();

        double magnitude = new Translation2d(yAxis, xAxis).getNorm();

        double angleDifference = Math.abs(new Translation2d(yAxis, xAxis).getAngle().minus(Swerve.get().getAngleForNote()).getDegrees());

        if (!Serializer.get().BackBeam() && magnitude > deadband && angleDifference < 20) {
            Swerve.get().Drive(new Translation2d(0.2 * SwerveConstants.maxSpeed, 0), 
                rotationController.calculate(Swerve.get().intakeCam.tX()), 
                false, true);
        } else {
            super.execute();
        }
    }
}
