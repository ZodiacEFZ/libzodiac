package frc.libzodiac;

/**
 * Common utilities.
 */
public class Util {
    public static double round(double value, int places) {
        if (places < 0) {
            throw new IllegalArgumentException();
        }
        long factor = (long) Math.pow(10, places);
        value = value * factor;
        long tmp = Math.round(value);
        return (double) tmp / factor;
    }

    public static String printed(double value) {
        final var x = Math.abs(value);
        if (x < 1e-6) {
            return "0";
        } else if (x < 1e-2) {
            return String.format("%.3e", value);
        } else if (x < 1) {
            return String.format("%.3f", value);
        } else if (x < 1000) {
            return String.format("%.1f", value);
        } else {
            return String.format("%.3e", value);
        }
    }

    public static boolean approx(double x0, double x1, double thre) {
        return Math.abs(x0 - x1) < thre;
    }

    public static boolean approx(double x0, double x1) {
        return approx(x0, x1, 1e-3);
    }

    public static double closer(double x, double x0, double x1) {
        return Math.abs(x - x0) < Math.abs(x - x1) ? x0 : x1;
    }

    public static Tuple2<Double, Integer> swerve_optimize(double curr, double angle) {
        var delta = angle - curr;
        return delta > Math.PI / 2 ? new Tuple2<>(closer(curr, angle - Math.PI, angle + Math.PI), -1) : new Tuple2<>(angle, 1);
    }

    public static class Tuple2<T0, T1> {
        public final T0 x0;
        public final T1 x1;

        public Tuple2(T0 x0, T1 x1) {
            this.x0 = x0;
            this.x1 = x1;
        }
    }
}
