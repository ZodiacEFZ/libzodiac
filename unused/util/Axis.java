package frc.libzodiac.unused.util;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

/**
 * A linear and continuous input, which could either be yielded by sensors or
 * controllers.
 */
public final class Axis {

    /**
     * The quadratic input filter, i.e. applies mapping x -> x^2 * sgn(x).
     */
    public static final Function<Double, Double> QUAD_FILTER = x -> x * x * Math.signum(x);

    /**
     * The arctangent input filter, which is a warped arctangent curve that maps
     * [-1,1] to [-1,1].
     */
    public static final Function<Double, Double> ATAN_FILTER = x -> (0.3771257645012 * Math.atan(8 * Math.abs(x) - 4)
            + 0.5) * Math.signum(x);

    /**
     * The mapping to apply to the original input.
     */
    public final Function<Double, Double> mapping;

    /**
     * Internal access to raw data source.
     */
    private final DoubleSupplier raw_input;

    private Axis(DoubleSupplier raw_input, Function<Double, Double> mapping) {
        this.raw_input = raw_input;
        this.mapping = mapping;
    }

    /**
     * Bind to a raw data source.
     *
     * @param raw_input the raw data input
     * @return new
     */
    public static Axis with(DoubleSupplier raw_input) {
        return new Axis(raw_input, Function.identity());
    }

    /**
     * Get the processed output.
     *
     * @return process output with `mapping` applied
     */
    public double get() {
        final var v = this.raw_input.getAsDouble();
        return this.mapping.apply(v);
    }

    /**
     * Get raw output.
     *
     * @return raw data output by the sensor
     */
    public double get_raw() {
        return this.raw_input.getAsDouble();
    }

    /**
     * Create an inverted axis based on current axis.
     *
     * @return new
     */
    public Axis inverted() {
        return this.map(x -> -x);
    }

    /**
     * Create a new <code>Axis</code> with <code>mapping</code> added.
     *
     * @param mapping the mapping to compose with current
     * @return new
     */
    public Axis map(Function<Double, Double> mapping) {
        final var f = this.mapping.compose(mapping);
        return this.with_map(f);
    }

    /**
     * Create a new <code>Axis</code> with specified <code>mapping</code>.
     *
     * @param mapping the mapping for the new axis
     * @return new
     */
    public Axis with_map(Function<Double, Double> mapping) {
        return new Axis(this.raw_input, mapping);
    }

    /**
     * Add a threshold filter to the current axis.
     *
     * @param thre the threshold
     * @return new
     */
    public Axis threshold(double thre) {
        final var f = thre_filter(thre);
        return this.map(f);
    }

    /**
     * Create a threshold filter that blocks inputs with absolute values less than
     * `thre`.
     *
     * @param thre the threshold
     * @return a filtering function
     */
    public static Function<Double, Double> thre_filter(double thre) {
        return x -> Math.abs(x) < thre ? 0 : x;
    }

    /**
     * Downcast the axis as a button, use 0.5 as threshold for absolute value.
     *
     * @return the button
     */
    public Button as_button() {
        return this.as_button(0.5);
    }

    /**
     * Downcast the axis as a button, use <code>thre</code> as threshold for
     * absolute value.
     *
     * @return the button
     */
    public Button as_button(double thre) {
        return Button.with(() -> Math.abs(this.raw_input.getAsDouble()) >= thre);
    }
}
