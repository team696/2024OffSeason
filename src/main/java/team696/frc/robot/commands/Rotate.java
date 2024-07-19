// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Swerve;
import team696.frc.robot.util.Constants;

public class Rotate extends Command {

  PIDController pidController;

  public Rotate() {        
      pidController = new PIDController(0.0056, 0.00, 0);
      pidController.enableContinuousInput(-180, 180);

      addRequirements(Swerve.get());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
        double rAxis = 0;
        
        double pid = pidController.calculate(Swerve.get().getPose().getRotation().getDegrees(), Swerve.get().getAngleToSpeaker().getDegrees());
        rAxis = Math.abs(Math.pow(pid, 2)) * 0.7 * Math.signum(pid) + pid * 2;

        Swerve.get().Drive(new Translation2d(), rAxis * Constants.swerve.maxAngularVelocity, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    Swerve.get().Drive(new Translation2d(), 0, true, true);
  }

  @Override
  public boolean isFinished() {
    if(Math.abs(Swerve.get().getPose().getRotation().getDegrees() - Swerve.get().getAngleToSpeaker().getDegrees()) < 3.)
        return true;
    return false;
  }
}