// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.Constants;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Shooter;

public class Amp extends Command {

  BooleanSupplier shoot_button;

  Constants.shooter.state desired = new Constants.shooter.state(12.25, 875, 850);

  public Amp(BooleanSupplier sbutton) {
    shoot_button = sbutton;

    addRequirements(Shooter.get(), Hood.get(), Serializer.get());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Shooter.get().setShooter(desired);
    Hood.get().setHood(desired);
    if (shoot_button.getAsBoolean() ) {
      if (Hood.get().getPosition() > desired.angle - 4.90) {
        Serializer.get().setSpeed(1.);
      } else {
        Serializer.get().setSpeed(0);
      }
    } else {
      Serializer.get().setSpeed(0);
    }

   /*  if (Serializer.get().BackBeam() && Serializer.get().FrontBeam()) {
      LED.get().setColor(0,255,0);
    } else {
      LED.get().setColor(255,0,0);
    } */
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
