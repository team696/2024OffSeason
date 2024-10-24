// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.Constants;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Shooter;
import team696.frc.robot.subsystems.Swerve;

public class Pass extends Command {

  boolean feed;

  boolean didSeeFront = false;
  double didUnseeFront = 0;

  public Pass() {
    addRequirements(Hood.get(), Serializer.get(), Shooter.get());
  }

  @Override
  public void initialize() {
    feed = false;

    didSeeFront = false;
    didUnseeFront = 0;
  }

  @Override
  public void execute() {
    double dist = Swerve.get().getDistToCorner();
    Constants.shooter.state desiredState = Constants.shooter.adjustedPassState(dist);

    Shooter.get().setShooter(desiredState);
    Hood.get().setHood(desiredState);

    if (Shooter.get().upToSpeed(desiredState, 250) 
        && Hood.get().atAngle(desiredState, 1.5)
        && Math.abs(Swerve.get().getPose().getRotation().getDegrees() - Swerve.get().getAngleToCorner().getDegrees()) < 5
        && Math.abs(Swerve.get().getRobotRelativeSpeeds().omegaRadiansPerSecond) < 0.3) {
          feed = true;
    }

    if (feed) {
      Serializer.get().setSpeed(1);
    } else {
      Serializer.get().stop();
    }

    if (feed) {
      if (!Serializer.get().FrontBeam()) {
        didSeeFront = true;
      }

      if (didSeeFront && didUnseeFront == 0 && Serializer.get().FrontBeam()) {
        didUnseeFront = Timer.getFPGATimestamp();
      }
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
    if (didUnseeFront != 0 && Timer.getFPGATimestamp() - didUnseeFront > 0.75) {
      return true;
    }
    if (!didSeeFront && Serializer.get().FrontBeam() && Serializer.get().BackBeam()) {
      return true;
    }

    return false;
  }
}
