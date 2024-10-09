package team696.frc.lib.Dashboards;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class ShuffleDashboard {
    private static ShuffleboardTab _mainTab; 

    public static final Field2d field = new Field2d();

    public final static void initialize() {
        _mainTab = Shuffleboard.getTab("Telemetry");

        _mainTab.add(field).withPosition(0, 0).withSize(10, 6);
    }

    public static ComplexWidget addObject(Sendable object) {
        return _mainTab.add(object);
    }

    public static ShuffleboardTab Tab() {
        return _mainTab;
    }

    public static void addAutoChooser(Sendable chooser) {
        _mainTab.add("Autos", chooser).withPosition(10, 0).withSize(3,1);
    }
}
