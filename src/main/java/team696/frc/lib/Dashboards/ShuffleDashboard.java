package team696.frc.lib.Dashboards;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleDashboard {
    private final static ShuffleboardTab _mainTab = Shuffleboard.getTab("Telemetry");

    public static ComplexWidget addObject(Sendable object) {
        return _mainTab.add(object);
    }

    public static ShuffleboardTab Tab() {
        return _mainTab;
    }
}
