// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Shooter;

public class ManualShot extends Command {

  boolean feed;
  Shooter.state desiredState;

  double extraadded = 0;

  public ManualShot(Shooter.state d) {
    desiredState = d;
    

    addRequirements(Hood.get(), Serializer.get(), Shooter.get());
  }

  @Override
  public void initialize() {
    feed = false;
  }

  @Override
  public void execute() {
    extraadded = 0;
    Shooter.state adesiredState = new Shooter.state(desiredState.angle + extraadded, desiredState.speed_l, desiredState.speed_r);
    Shooter.get().setShooter(adesiredState);
    Hood.get().setHood(adesiredState);

    if (Shooter.get().upToSpeed(adesiredState, 150) 
        && Hood.get().atAngle(adesiredState, 1.5)) {
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
    //if (Serializer.get().FrontBeam())
      //return true;

    return false;
  }
}
