package frc.libzodiac;

import java.util.function.Function;

import edu.wpi.first.wpilibj.Timer;

/**
 * Common utilities.
 */
public class Util {
    public static void blocking_wait(int ms) {
        final var t = new Timer();
        while (t.get() < ((double) ms) / 1000)
            ;
    }

    /**
     * Convert degrees to radians.
     */
    public static double rad(double deg) {
        return deg / 180 * Math.PI;
    }

    /**
     * Convert radians to degrees.
     */
    public static double deg(double rad) {
        return rad / Math.PI * 180;
    }

    public static double round(double value, int places) {
        if (places < 0)
            throw new IllegalArgumentException();
        long factor = (long) Math.pow(10, places);
        value = value * factor;
        long tmp = Math.round(value);
        return (double) tmp / factor;
    }

    public static String printed(double value) {
        final var x = Math.abs(value);
        if (x < 1e-6)
            return "0";
        else if (x < 1e-2)
            return String.format("%.3e", value);
        else if (x < 1)
            return String.format("%.3f", value);
        else if (x < 1000)
            return String.format("%.1f", value);
        else
            return String.format("%.3e", value);
    }

    /**
     * Extends the modulo operation to R.
     * <br/>
     * The definition here:
     * <br/>
     * a and b are congruent modulo c, if and only if (a-b)/c is integer.
     *
     * @return In (-mod,mod), and NaN for NaN and Infinite parameters.
     */
    public static double mod(Double num, double mod) {
        // I believe there are still some bugs.
        if (num.isInfinite() || num.isNaN())
            return Double.NaN;
        if (num < 0) {
            return -mod(-num, mod);
        }
        final var ans = (int) (num / 2 / mod);
        final var res = num - ans * 2 * mod;
        return res > mod ? res - 2 * mod : res;
    }

    /**
     * Convert a radian into (-pi,pi).
     */
    public static double mod_pi(double rad) {
        return mod(rad, Math.PI);
    }

    public static double induct(double angle) {
        if (angle < 0)
            return -induct(-angle);
        final var cos = Math.cos(angle);
        final var acos = Math.acos(cos);
        return acos;
    }

    public static double closest(double angle) {
        if (angle < 0)
            return -closest(-angle);
        final var cnt = (int) (angle / (2 * Math.PI));
        final var one = cnt * 2 * Math.PI;
        return angle - one > Math.PI ? one + 2 * Math.PI : one;
    }

    public static final Function<Double, Double> abs_thre(double thre) {
        return x -> Math.abs(x) < thre ? 0 : x;
    }

    public static Tuple2<Double, Boolean> solve(double src, double dst) {
        // var mod_pied = mod_pi(src);
        // var delta = mod_pi(dst - mod_pied);
        // boolean inverted = false;
        // if(Math.abs(delta)>Math.PI/2)
        // {
        // mod_pied = mod_pi(mod_pied+Math.PI);
        // inverted = true;
        // }
        // delta = mod_pi(dst - mod_pied);
        // return new Tuple2<Double,Boolean>(src + delta, inverted);

        final var closest = closest(src);
        final var d0 = closest + dst;
        final var d1 = closest + dst - Math.PI;
        final var d2 = closest + dst + Math.PI;
        final var a0 = Math.abs(src - d0);
        final var a1 = Math.abs(src - d1);
        final var a2 = Math.abs(src - d2);

        final var min = Math.min(a0, Math.min(a1, a2));
        return min == a0 ? new Tuple2<>(d0, false)
                : min == a1 ? new Tuple2<>(d1, true)
                        : new Tuple2<>(d2, true);

        // final var d1 = closest + dst;
        // final var absdelta = Math.abs(d1 - src);
        // final var inverted = absdelta >= Math.PI / 2;
        // final var d2 = (inverted || Math.abs(absdelta - Math.PI) < 1e-3) ? (d1 < src
        // ? d1 + Math.PI : d1 - Math.PI) : d1;
        // return new Tuple2<>(d2, inverted);
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
