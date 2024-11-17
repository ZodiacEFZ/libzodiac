package frc.libzodiac;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import java.util.HashMap;

public class ZDashboard {
    private static ShuffleboardTab tab = Shuffleboard.getTab("Debug");

    private ZDashboard(String name) {
        ZDashboard.selectTab(name);
    }

    public static ShuffleboardTab selectTab(String name) {
        ZDashboard.tab = Shuffleboard.getTab(name);
        Shuffleboard.selectTab(name);
        return ZDashboard.tab;
    }

    public static ShuffleboardTab tab(String name) {
        return Shuffleboard.getTab(name);
    }

    public static Widget add(String name, Object value) {
        if (WidgetStorage.exists(name)) {
            WidgetStorage.get(name).getEntry().setValue(value);
            return new Widget(null);
        }
        final var widget = new Widget(ZDashboard.tab.add(name, value));
        WidgetStorage.add(name, widget);
        return widget;
    }

    public static class Widget {
        private final SimpleWidget widget;

        Widget(SimpleWidget widget) {
            this.widget = widget;
        }

        public SimpleWidget getWidget() {
            return widget;
        }

        public GenericEntry getEntry() {
            return widget.getEntry();
        }
    }

    private static class WidgetStorage {
        private static final HashMap<String, HashMap<String, Widget>> map = new HashMap<>();

        public static boolean exists(String name) {
            return WidgetStorage.exists(ZDashboard.tab.getTitle(), name);
        }

        public static boolean exists(String tabName, String name) {
            return map.containsKey(tabName) && map.get(tabName).containsKey(name);
        }

        public static Widget get(String name) {
            return WidgetStorage.get(ZDashboard.tab.getTitle(), name);
        }

        public static Widget get(String tabName, String name) {
            if (map.containsKey(tabName)) {
                if (map.get(tabName).containsKey(name)) {
                    return map.get(tabName).get(name);
                }
            }
            return null;
        }

        public static Widget add(String name, Widget widget) {
            return WidgetStorage.add(ZDashboard.tab.getTitle(), name, widget);
        }

        public static Widget add(String tabName, String name, Widget widget) {
            if (map.containsKey(tabName)) {
                map.get(tabName).put(name, widget);
            } else {
                map.put(tabName, new HashMap<>());
                map.get(tabName).put(name, widget);
            }
            return widget;
        }
    }

    public static class Widgets
    {
    }
}

