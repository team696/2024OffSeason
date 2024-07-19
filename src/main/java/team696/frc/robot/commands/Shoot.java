// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.LED;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Shooter;
import team696.frc.robot.subsystems.Swerve;
import team696.frc.robot.util.Constants;

public class Shoot extends Command {

  boolean feed;

  boolean didSeeFront = false;
  double didUnseeFront = 0;

  public Shoot() {
    addRequirements(Hood.get(), Serializer.get(), Shooter.get(), LED.get());
  }

  @Override
  public void initialize() {
    feed = false;

    didSeeFront = false;
    didUnseeFront = 0;
  }

  @Override
  public void execute() {
    double dist = Swerve.get().getDistToSpeaker();
    Constants.shooter.state desiredState = Constants.shooter.adjustedState(dist);

    Shooter.get().setShooter(desiredState);
    Hood.get().setHood(desiredState);

    if (Shooter.get().upToSpeed(desiredState, 100) 
        && Hood.get().atAngle(desiredState, 1.5)
        && Math.abs(Swerve.get().getPose().getRotation().getDegrees() - Swerve.get().getAngleToSpeaker().getDegrees()) < 6
        && Math.abs(Swerve.get().getRobotRelativeSpeeds().omegaRadiansPerSecond) < 1
        && Math.abs(Swerve.get().getRobotRelativeSpeeds().vxMetersPerSecond) < 1.5
        && Math.abs(Swerve.get().getRobotRelativeSpeeds().vyMetersPerSecond) < 3) {
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

    if(Serializer.get().FrontBeam() && Serializer.get().FrontBeam()) {
      LED.get().setColor(0, 255, 0);
    } else {
      LED.get().setColor(255, 0, 0);
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
    if (didUnseeFront != 0 && Timer.getFPGATimestamp() - didUnseeFront > 0.01) {
      return true;
    }
    if (!didSeeFront && Serializer.get().FrontBeam() && Serializer.get().BackBeam()) {
      return true;
    }

    return false;
  }
}
