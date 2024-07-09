// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Shooter;
import team696.frc.robot.util.Constants;

public class Amp extends Command {

  BooleanSupplier shoot_button;

  Constants.shooter.state desired = new Constants.shooter.state(11., 1300, 900);

  public Amp(BooleanSupplier sbutton) {
    shoot_button = sbutton;

    addRequirements(Shooter.get(), Hood.get(), Serializer.get());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Shooter.get().upToSpeed(desired,125)) {
      if (shoot_button.getAsBoolean() ) {
        Hood.get().setHood(desired);
        if (Hood.get().atAngle(desired, 3.5)) {
                Serializer.get().setSpeed(1.);
        } else {
        Serializer.get().setSpeed(0);
        }
      }
    } else {
      Hood.get().setHood(0);
    }

    
    Shooter.get().setShooter(desired);


  }

  @Override
  public void end(boolean interrupted) {
    Hood.get().stop();
    Shooter.get().stop();
    Serializer.get().stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
