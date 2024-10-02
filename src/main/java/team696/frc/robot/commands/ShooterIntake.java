// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.LED;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Shooter;

public class ShooterIntake extends Command {
  boolean hasSeenFront;
  boolean hasUnSeenFront;
  boolean shouldEnd;

  public ShooterIntake() {
    addRequirements(Shooter.get(), Serializer.get(), Hood.get(), LED.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasSeenFront = false;
    hasUnSeenFront = false;
    shouldEnd = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!Serializer.get().FrontBeam()) {
      hasSeenFront = true;
    }
    if (hasSeenFront && Serializer.get().FrontBeam()) {
      hasUnSeenFront = true;
    }
    if(hasSeenFront && hasUnSeenFront && !Serializer.get().FrontBeam()) {
      shouldEnd = true;
      Hood.get().setHood(0);
      return;
    }

    Hood.get().setHood(11.);

    if (Serializer.get().BackBeam() || !Serializer.get().FrontBeam()) {
      double multipliter = 1;
      if (!Serializer.get().FrontBeam()) multipliter = 0.5;
      Shooter.get().setShooter(-1800,-1800);
      Serializer.get().setSpeed(-0.6 * multipliter);
    } else {
      Shooter.get().stop();
      Serializer.get().setSpeed(0.2);
    }

    if (!Serializer.get().FrontBeam() || !Serializer.get().BackBeam()) {
      LED.get().setColor(0,255,0);
    } else {
      LED.get().setColor(255,0,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.get().stop();
    Serializer.get().stop();
    Hood.get().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd;
  }
}
