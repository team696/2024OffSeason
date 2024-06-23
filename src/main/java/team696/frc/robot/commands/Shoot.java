// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Shooter;
import team696.frc.robot.subsystems.Swerve;
import team696.frc.robot.util.Constants;

public class Shoot extends Command {

  boolean feed;

  public Shoot() {
    addRequirements(Hood.get(), Serializer.get(), Shooter.get());
  }

  @Override
  public void initialize() {
    feed = false;
  }

  @Override
  public void execute() {
    double dist = Swerve.get().getDistToSpeaker();
    Constants.shooter.state desiredState = Constants.shooter.adjustedState(dist);

    Shooter.get().setShooter(desiredState);
    Hood.get().setHood(desiredState);

    if (Shooter.get().upToSpeed(desiredState, 200) 
        && Hood.get().atAngle(desiredState, 1)
        && Math.abs(Swerve.get().getPose().getRotation().getDegrees() - Swerve.get().getAngleToSpeaker().getDegrees()) < 4) {
          feed = true;
    }

    if (feed) {
      Serializer.get().setSpeed(1);
    } else {
      Serializer.get().stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    Shooter.get().stop();
    Serializer.get().stop();
    Hood.get().stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
