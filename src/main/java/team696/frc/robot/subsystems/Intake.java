// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team696.frc.lib.HardwareDevices.TalonFactory;
import team696.frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static Intake _Intake;

  private TalonFactory _serializer;

  /** Creates a new Intake. */
  private Intake() {
    _serializer = new TalonFactory(10, Constants.canivoreName, Constants.configs.intake.serializer, "Intake Serializer");
  }

  public static Intake get(){
    if (_Intake == null) {
      _Intake = new Intake();
    }

    return _Intake;
  }

  public void stop() {
    _serializer.stop();
  }

  public void setSpeed(double speed) {
    _serializer.PercentOutput(speed);
  }

  public Command spin(double speed) {
    return this.runEnd(()->setSpeed(speed), ()->_serializer.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
