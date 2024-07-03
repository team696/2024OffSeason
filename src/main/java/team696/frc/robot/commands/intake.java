// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.Intake;
import team696.frc.robot.subsystems.Serializer;

public class intake extends Command {
  
  public intake() {
    addRequirements(Hood.get(), Intake.get(), Serializer.get());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Hood.get().setHood(6);
    Serializer.get().serialize();

    if (Hood.get().getPosition() > 5) {
      Intake.get().setSpeed(0.87);
    } else {
      Intake.get().stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    Hood.get().stop();
    Intake.get().stop();
    Serializer.get().stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
