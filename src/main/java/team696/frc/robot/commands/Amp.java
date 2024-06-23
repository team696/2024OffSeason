// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Shooter;

public class Amp extends Command {

  BooleanSupplier shoot_button;

  public Amp(BooleanSupplier sbutton) {
    shoot_button = sbutton;

    addRequirements(Shooter.get(), Hood.get(), Serializer.get());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Hood.get().setHood(8);
    
    Shooter.get().setShooter(500, 500);

    if (shoot_button.getAsBoolean()) {
      Serializer.get().setSpeed(0.75);
    } else {
      Serializer.get().setSpeed(0);
    }
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
