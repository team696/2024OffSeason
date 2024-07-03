// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team696.frc.lib.TalonFactory;
import team696.frc.robot.util.Constants;

public class Serializer extends SubsystemBase {
  private static Serializer m_Serializer;

  private TalonFactory _Serializer;

  private DigitalInput _FrontBeam;
  private DigitalInput _BackBeam;

  private PositionVoltage _PositionController;

  /** Creates a new Serializer. */
  private Serializer() {
    _Serializer = new TalonFactory(15, Constants.canivoreName, Constants.configs.shooter.serializer, "Shooter Serializer");

    _FrontBeam = new DigitalInput(1);
    _BackBeam = new DigitalInput(0);

    _PositionController = new PositionVoltage(0);

    this.setDefaultCommand(holdPosition());
  }

  public static Serializer get() {
    if (m_Serializer == null) {
      m_Serializer = new Serializer();
    }

    return m_Serializer;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    _Serializer.PercentOutput(speed);
  }

  public void stop() {
    _Serializer.stop();
  }

  public Command feed(double speed) {
    return this.runEnd(()->setSpeed(speed), ()->_Serializer.stop());
  }

  public boolean FrontBeam() {
    return _FrontBeam.get();
  }

  public boolean BackBeam() {
    return _BackBeam.get();
  }

  public void serialize() {
    if (!FrontBeam()) {
        _Serializer.stop();
      } else {
        if (!BackBeam()) {
          setSpeed(0.25);
        } else {
          setSpeed(0.65);
        }
      }
  }

  public Command holdPosition() {
    return this.startEnd(()->_Serializer.setControl(_PositionController.withPosition(_Serializer.getPosition())), ()->_Serializer.stop());
  }

  public Command intake() {
    return this.runEnd(this::serialize, ()->_Serializer.stop());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Front Beam Break", ()->FrontBeam(), null);
    builder.addBooleanProperty("Back Beam Break", ()->BackBeam(), null);
  }
}
