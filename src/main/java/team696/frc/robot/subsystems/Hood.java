// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team696.frc.lib.TalonFactory;
import team696.frc.robot.util.Constants;

public class Hood extends SubsystemBase {

    private static Hood m_Hood;

    private TalonFactory _LeftAngle;
    private TalonFactory _RightAngle;

    private MotionMagicVoltage _AngleRequest;

    private double inputangle;

    public static Hood get() {
        if (m_Hood == null) {
            m_Hood = new Hood();
        }

        return m_Hood;
    }

    /** Creates a new Hood. */
    private Hood() {
        _LeftAngle = new TalonFactory(
            11,
            Constants.canivoreName,
            Constants.configs.hood.left,
            "Shooter Left Angle"
        );
        _RightAngle = new TalonFactory(
            12,
            Constants.canivoreName,
            Constants.configs.hood.right,
            "Shooter Right Angle"
        );

        _RightAngle.Follow(_LeftAngle, true);

        _LeftAngle.setPosition(0);
        _RightAngle.setPosition(0);

        _AngleRequest = new MotionMagicVoltage(0);

        this.setDefaultCommand(this.positionHood(0.1));
    }

    public double getPosition() {
        return _LeftAngle.getPosition();
    }

    public void setHood(Constants.shooter.state desired) { 
        setHood(desired.angle);
    }

    public void setHood(double angle) {                                              
        _LeftAngle.setControl(_AngleRequest
                                .withPosition(angle)
                                //.withFeedForward(12 * Constants.shooter.kG * Math.cos(Util.map(getPosition(), 0.0, 8., 0.88, Math.PI/2)))
                             ); 
    }

    public void stop() {
        _LeftAngle.stop();
    }

    public boolean atAngle(Constants.shooter.state desired, double tolerance) {
        if (
            desired.angle < getPosition() - tolerance ||
            desired.angle > getPosition() + tolerance
        ) return false;

        return true;
    }

    public Command positionHood(double pos) {
        return this.runEnd(
                () -> setHood(pos),
                () -> _LeftAngle.stop()
            );
    }

    public Command positionHood() {
        return this.runEnd(
                () -> _LeftAngle.setControl(_AngleRequest.withPosition(inputangle)),
                () -> _LeftAngle.stop()
            );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Left angle",
            () -> _LeftAngle.getPosition(),
            null
        );
        builder.addDoubleProperty(
            "Right angle",
            () -> _RightAngle.getPosition(),
            null
        );

        builder.addDoubleProperty("Manual Angle", null, a -> inputangle = a);
        SmartDashboard.putNumber("Hood/Manual Angle", inputangle);
    }
}
