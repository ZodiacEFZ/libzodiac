package frc.libzodiac.util;

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

    /**
     * Prettify a float point number to a moderate length to print on the screen.
     * 
     * @param value the float point number
     * @return formatted string
     */
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

    /**
     * Check if two float point numbers are approximately equal. The threshold for
     * difference of absolute value is 1e-3.
     * 
     * @param lhs a float point number
     * @param rhs another float point number
     * @return result of comparison
     */
    public static boolean approx(double lhs, double rhs) {
        return approx(lhs, rhs, 1e-3);
    }

    /**
     * Check if two float point numbers are approximately equal.
     * 
     * @param lhs  a float point number
     * @param rhs  another float point number
     * @param thre threshold for difference of absolute value
     * @return result of comparison
     */
    public static boolean approx(double lhs, double rhs, double thre) {
        return Math.abs(lhs - rhs) < thre;
    }

    /**
     * Check which number is closer to a specified number.
     * 
     * @param base the specified number
     * @param x    a number to compare with
     * @param y    another number to compare with
     * @return the "closer" number
     */
    public static double closer(double base, double x, double y) {
        return Math.abs(base - x) < Math.abs(base - y) ? x : y;
    }
}
