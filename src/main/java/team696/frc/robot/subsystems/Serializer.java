// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team696.frc.lib.HardwareDevices.TalonFactory;
import team696.frc.robot.Constants;

public class Serializer extends SubsystemBase {
  private static Serializer _Serializer;

  private TalonFactory _motor;

  private DigitalInput _frontBeam;
  private DigitalInput _backBeam;

  private PositionVoltage _positionController;

  /** Creates a new Serializer. */
  private Serializer() {
    _motor = new TalonFactory(15, Constants.canivoreName, Constants.configs.shooter.serializer, "Shooter Serializer");

    _frontBeam = new DigitalInput(3);
    _backBeam = new DigitalInput(4);

    _positionController = new PositionVoltage(0);

    this.setDefaultCommand(holdPosition());
  }

  public static Serializer get() {
    if (_Serializer == null) {
      _Serializer = new Serializer();
    }

    return _Serializer;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    _motor.VoltageOut(speed);
  }

  public void stop() {
    _motor.stop();
  }

  public Command feed(double speed) {
    return this.runEnd(()->setSpeed(speed), ()->_motor.stop());
  }

  public boolean FrontBeam() {
    return _frontBeam.get();
  }

  public boolean BackBeam() {
    return _backBeam.get();
  }

  public void serialize() {
    if (!FrontBeam()) {
        _motor.stop();
      } else {
        if (!BackBeam()) {
          setSpeed(0.15);
        } else {
          setSpeed(0.4);
        }
      }
  }

  public Command holdPosition() {
    return this.startEnd(()->_motor.setControl(_positionController.withPosition(_motor.getPosition())), ()->_motor.stop());
  }

  public Command intake() {
    return this.runEnd(this::serialize, ()->_motor.stop());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Front Beam Break", this::FrontBeam, null);
    builder.addBooleanProperty("Back Beam Break", this::BackBeam, null);

    builder.addDoubleProperty("Serializer Position", _motor::getPosition, null);
  }
}
