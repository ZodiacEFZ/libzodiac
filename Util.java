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

    public static double closer(double x0, double x1, double x2) {
        return Math.abs(x0 - x1) < Math.abs(x0 - x2) ? x1 : x2;
    }

    public static boolean approx(double x0, double x1, double thre) {
        return Math.abs(x0 - x1) < thre;
    }

    public static boolean approx(double x0, double x1) {
        return approx(x0, x1, 1e-3);
    }
}
