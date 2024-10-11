package team696.frc.lib.HardwareDevices;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import team696.frc.lib.PLog;

public class PigeonFactory implements GyroInterface {

    private final double TIMEOUT = 0.05;
    private Pigeon2 _gyro;
    private Pigeon2Configuration _config;
    private String _name;

    private boolean _configured = false;
    private double _lastConfiguration = -100;

    public PigeonFactory(int id, String canBus, Pigeon2Configuration config, String name) {
        this._gyro = new Pigeon2(id, canBus);
        this._name = name;
        this._config = config;
        configure();
    }

    public PigeonFactory(int id, Pigeon2Configuration config, String name) {
        this(id, "rio", config, name);
    }

    private boolean configure() {
        return configure(false);
    }

        public boolean configure(boolean force) {
        if (!force && _configured) return true;
        if (!force && Timer.getFPGATimestamp() - _lastConfiguration < 3) return false;

        _lastConfiguration = Timer.getFPGATimestamp();
        StatusCode configCode = _gyro.getConfigurator().apply(this._config, TIMEOUT);

        if(configCode.isError()) {
            PLog.unusual(_name, "Failed to configure");
        } else {
            PLog.info(_name, "Configured");
            if (configCode.isWarning()) {
                PLog.unusual(_name, "Config Warning: " + configCode.toString());
            }

            _configured = true;
        }

        return _configured;
    }

    private double getRawYaw() {
        if (configure()) {
            StatusSignal<Double> positionCode = _gyro.getYaw();
            if(!positionCode.getStatus().isOK()) {
                _configured = false;
                return 0;
            }
            return MathUtil.inputModulus(positionCode.getValueAsDouble(),-180,180);
        } else 
            return 0;
    }

    /*
     * Angle Of the Gyro in -180, 180
     */
    @Override 
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(getRawYaw());
    }

    @Override 
    public void resetYaw() {
        _gyro.reset();
    }

    @Override 
    public double getAngularVelocity() {
        if (configure()) {
            StatusSignal<Double> positionCode = _gyro.getAngularVelocityZWorld();
            if(!positionCode.getStatus().isOK()) {
                _configured = false;
                return 0;
            }
            return positionCode.getValueAsDouble();
        } else 
            return 0;
    }

    public Pigeon2 get() {
        return _gyro;
    }

    public Rotation2d getLatencyAdjustedYaw() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(BaseStatusSignal.getLatencyCompensatedValue(_gyro.getYaw(), _gyro.getAngularVelocityZWorld()),-180,180));
    }
}
