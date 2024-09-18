package frc.libzodiac.ui;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public final class Axis {
    public static final Function<Double, Double> QUAD_FILTER = x -> x * x * Math.signum(x);
    public static final Function<Double, Double> ATAN_FILTER = x -> (0.3771257645012 * Math.atan(8 * Math.abs(x) - 4) + 0.5) * Math.signum(x);
    public final Function<Double, Double> mapping;
    private final DoubleSupplier raw_input;

    public Axis(DoubleSupplier raw_input) {
        this.raw_input = raw_input;
        this.mapping = x -> x;
    }

    public Axis(DoubleSupplier raw_input, Function<Double, Double> mapping) {
        this.raw_input = raw_input;
        this.mapping = mapping;
    }

    public static Function<Double, Double> thre_filter(double thre) {
        return x -> Math.abs(x) < thre ? 0 : x;
    }

    /**
     * Get the processed output.
     */
    public double get() {
        final var v = this.raw_input.getAsDouble();
        return this.mapping.apply(v);
    }

    /**
     * Get raw output.
     */
    public double get_raw() {
        return this.raw_input.getAsDouble();
    }

    /**
     * Create a new <code>Axis</code> with specified <code>mapping</code>.
     */
    public Axis with_map(Function<Double, Double> mapping) {
        return new Axis(this.raw_input, mapping);
    }

    /**
     * Create a new <code>Axis</code> with <code>mapping</code> added.
     */
    public Axis map(Function<Double, Double> mapping) {
        final var f = this.mapping.compose(mapping);
        return this.with_map(f);
    }

    public Axis inverted() {
        return this.map(x -> -x);
    }

    public Axis threshold(double thre) {
        final var f = thre_filter(thre);
        return this.map(f);
    }

    public Button as_button() {
        return new Button(() -> Math.abs(this.raw_input.getAsDouble()) >= 0.5);
    }

    public Button as_button(double thre) {
        return new Button(() -> Math.abs(this.raw_input.getAsDouble()) >= thre);
    }
}
