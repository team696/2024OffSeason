// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.Intake;
import team696.frc.robot.subsystems.LED;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Swerve;

public class GroundIntake extends Command {
  
  public GroundIntake() {
    addRequirements(Hood.get(), Intake.get(), Serializer.get(), LED.get());
  }

  @Override
  public void initialize() {
    PPHolonomicDriveController.setRotationTargetOverride(()->Swerve.get().getAngleForNote() == null ? Optional.empty() : Optional.of(Swerve.get().getAngleForNote()));
  }

  @Override
  public void execute() {
    Hood.get().setHood(7);
    Serializer.get().serialize();

    if (Hood.get().getPosition() > 5) {
      if ( Serializer.get().BackBeam() ) {
        Intake.get().setSpeed(0.6);
      } else {
        Intake.get().setSpeed(0.2);
      }
    } else {
      Intake.get().stop();
    }

    if (!Serializer.get().FrontBeam()) {
      LED.get().setColor(0,255,0);
    } else if (!Serializer.get().BackBeam()) {
      LED.get().setColor(0,0,255); 
    } else {
      LED.get().setColor(255,0,0); 
    }
  }

  @Override
  public void end(boolean interrupted) {
    Hood.get().stop();
    Intake.get().stop();
    Serializer.get().stop();
    PPHolonomicDriveController.setRotationTargetOverride(()->Optional.empty());

  }

  @Override
  public boolean isFinished() {
    if (!Serializer.get().FrontBeam() && !Serializer.get().BackBeam())
      return true;

    return false;
  }
}

