package frc.libzodiac.ui;

import frc.libzodiac.util.Vec2;

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

    public Vec2 vec() {
        var v = new Vec2(this.x.get(), this.y.get());
        return v.with_r(this.mapping.apply(v.r()));
    }

    public Axis r() {
        return new Axis(this.vec()::r);
    }

    public Axis2D inverted() {
        return new Axis2D(this.x.inverted(), this.y.inverted(), this.mapping);
    }

    public Axis2D inverted(boolean xi, boolean yi) {
        return new Axis2D(xi ? this.x.inverted() : this.x, yi ? this.y.inverted() : this.y, this.mapping);
    }

    public Axis2D switched() {
        //noinspection SuspiciousNameCombination
        return new Axis2D(this.y, this.x, mapping);
    }

    public Axis2D threshold(double threshold) {
        return new Axis2D(this.x.threshold(threshold), this.y.threshold(threshold));
    }

    public Axis2D with_map(Function<Double, Double> mapping) {
        return new Axis2D(this.x, this.y, mapping);
    }

    public Axis2D map(Function<Double, Double> mapping) {
        final var f = this.mapping.compose(mapping);
        return this.with_map(f);
    }
}
