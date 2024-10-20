package frc.libzodiac;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ZDashboard {
    private static final ZDashboard dashboard = new ZDashboard("Debug");
    private ShuffleboardTab tab;

    public ZDashboard(String name) {
        tab = Shuffleboard.getTab(name);
        Shuffleboard.selectTab(name);
    }

    public ShuffleboardTab tab(String name) {
        return Shuffleboard.getTab(name);
    }

    public ShuffleboardTab selectedTab(String name) {
        tab = Shuffleboard.getTab(name);
        Shuffleboard.selectTab(name);
        return tab;
    }

    public interface Dashboard {
        default ZDashboard dashboard() {
            return ZDashboard.dashboard;
        }

        default ShuffleboardTab dashboardTab() {
            return ZDashboard.dashboard.tab;
        }
    }
}

