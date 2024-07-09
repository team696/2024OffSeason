// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Intake;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Shooter;

public class Drop extends Command {
  /** Creates a new Drop. */
  public Drop() {
    addRequirements(Shooter.get(), Serializer.get(), Intake.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.get().setShooterPercent(1,1);
    Serializer.get().setSpeed(-1);
    Intake.get().setSpeed(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.get().stop();
    Serializer.get().stop();
    Intake.get().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
