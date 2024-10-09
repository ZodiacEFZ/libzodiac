package frc.libzodiac.ui;

import frc.libzodiac.util.Vec2D;

import java.util.function.Function;

public class Axis2D {
    public final Function<Double, Double> mapping;
    private final Axis x, y;

    Axis2D(Axis x, Axis y) {
        this.x = x;
        this.y = y;
        this.mapping = Function.identity();
    }

    Axis2D(Axis x, Axis y, Function<Double, Double> mapping) {
        this.x = x;
        this.y = y;
        this.mapping = mapping;
    }

    public Vec2D vec() {
        var v = new Vec2D(x.get(), y.get());
        return v.with_r(this.mapping.apply(v.r()));
    }

    public Axis r() {
        return new Axis(this.vec()::r);
    }

    public Axis2D inverted() {
        return new Axis2D(x.inverted(), y.inverted());
    }

    public Axis2D threshold(double threshold) {
        return new Axis2D(x.threshold(threshold), y.threshold(threshold));
    }

    public Axis2D with_map(Function<Double, Double> mapping) {
        return new Axis2D(this.x, this.y, mapping);
    }

    public Axis2D map(Function<Double, Double> mapping) {
        final var f = this.mapping.compose(mapping);
        return this.with_map(f);
    }
}
