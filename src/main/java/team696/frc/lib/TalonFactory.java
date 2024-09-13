package team696.frc.lib;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;

public class TalonFactory { //TODO: Make this general for CAN devices
    private final double TIMEOUT = 0.05;
    private TalonFX m_Motor;
    private TalonFXConfiguration m_Config;
    private String name;
    private DutyCycleOut m_DutyCycleControl;
    private VoltageOut m_VoltageControl;

    private int followID = -1;
    private boolean opposeDirection = false;

    private boolean configured = false;
    private double lastConfigure = -100;

    private static List<TalonFactory> Motors = new ArrayList<TalonFactory>();
    private static Orchestra orchestra = new Orchestra();

    public TalonFactory(int id, String canBus, TalonFXConfiguration config, String name) {
        this.m_Motor = new TalonFX(id, canBus);
        this.name = name;
        this.m_Config = config;
        m_DutyCycleControl = new DutyCycleOut(0);
        m_VoltageControl = new VoltageOut(0);
        configure();
    }

    public TalonFactory(int id, TalonFXConfiguration config, String name) {
        this.m_Motor = new TalonFX(id);
        this.name = name;
        this.m_Config = config;
        m_DutyCycleControl = new DutyCycleOut(0);
        m_VoltageControl = new VoltageOut(0);
        configure();
    }

    private boolean configure() {
        return configure(false);
    }

    public boolean configure(boolean force) {
        if (!force && configured) return true;
        if (!force && Timer.getFPGATimestamp() - lastConfigure < 3) return false;

        lastConfigure = Timer.getFPGATimestamp();
        StatusCode configCode = m_Motor.getConfigurator().apply(this.m_Config, TIMEOUT);

        if(configCode.isError()) {
            PLog.unusual(name, "Failed to configure");
        } else {
            PLog.info(name, "Configured");
            if (configCode.isWarning()) {
                PLog.unusual(name, "Config Warning: " + configCode.toString());
            }

            if (followID != -1)
                m_Motor.setControl(new Follower(followID, opposeDirection));

            setPosition(0);

            configured = true;
        }

        return configured;
    }

    public static void configAll() {
        for (TalonFactory talon : Motors) {
            talon.configure();
        }
    }

    public void Follow(int id, boolean opposeDirection) {
        followID = id;
        this.opposeDirection = opposeDirection;
        if (configure())
            m_Motor.setControl(new Follower(id, opposeDirection)); 
    }

    public void Follow(TalonFX master, boolean opposeDirection) {
        Follow(master.getDeviceID(), opposeDirection);
    }

    public void Follow(TalonFactory master, boolean opposeDirection) {
        Follow(master.getID(), opposeDirection);
    }

    public int getID() {
        return m_Motor.getDeviceID();
    }

    public void setPosition(double newPosition) {
        if (configure())
            if(!m_Motor.setPosition(newPosition).isOK()) {
                PLog.info(this.name, "Failed To Set Position");
                configured = false;
            }
    }

    public void setPosition() {
        setPosition(0);
    }

    public void play() {
        orchestra.clearInstruments();

        for (TalonFactory motor : Motors) {
            if (motor.configure())
                orchestra.addInstrument(motor.m_Motor);
        }

        orchestra.loadMusic("beep.chrp");
        orchestra.play();
    }

    public double getPosition() {
        if (configure()) {
            StatusSignal<Double> positionCode = m_Motor.getPosition();
            if(!positionCode.getStatus().isOK()) {
                configured = false;
                return 0;
            }
            return positionCode.getValueAsDouble();
        } else 
            return 0;
    }

    public double getVelocity() {
        if (configure()) {
            StatusSignal<Double> velocityCode = m_Motor.getVelocity();
            if(!velocityCode.getStatus().isOK()) {
                configured = false;
                return 0;
            }
            return velocityCode.getValueAsDouble();
        } else 
            return 0;
    }

    public void PercentOutput(double percent) {
        setControl(m_DutyCycleControl.withOutput(percent));
    }

    /** @param voltage 0-1 output */
    public void VoltageOut(double voltage) {
        setControl(m_VoltageControl.withOutput(voltage * 12));
    }

    public void stop() {
        if (configure())
            m_Motor.stopMotor();
    }
    
    public void setControl(ControlRequest request) {
        if (configure())
            if (!m_Motor.setControl(request).isOK()) {
                configured = false;
            }
    }

    public double getCurrent() {
        if (configure()) {
            StatusSignal<Double> currentCode = m_Motor.getStatorCurrent();
            if(!currentCode.getStatus().isOK()) {
                configured = false;
                return 0;
            }
            return currentCode.getValueAsDouble();
        } else 
            return 0;
    }

    public TalonFX get() {
        return m_Motor;
        
    }
}
