package frc.libzodiac.util;

import frc.libzodiac.Util;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Translation2d;

public record Vec2D(double x, double y) {
    public double x() {
        return this.x;
    }

    public double y() {
        return this.y;
    }

    public double r() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public double theta() {
        return Math.atan2(this.y, this.x);
    }

    public static Vec2D polar(double r, double theta) {
        return new Vec2D(r * Math.cos(theta), r * Math.sin(theta));
    }

    @Override
    public String toString() {
        return "(" + Util.printed(this.x) + "," + Util.printed(this.y) + ")";
    }

    public Vec2D with_x(double x) {
        return new Vec2D(x, this.y);
    }

    public Vec2D with_y(double y) {
        return new Vec2D(this.x, y);
    }

    public Vec2D with_r(double r) {
        return polar(r, this.theta());
    }

    public Vec2D with_theta(double theta) {
        return polar(this.r(), theta);
    }

    public Vec2D inv() {
        return new Vec2D(-this.x, -this.y);
    }

    public Vec2D add(Vec2D rhs) {
        return new Vec2D(this.x + rhs.x, this.y + rhs.y);
    }

    public Vec2D sub(Vec2D rhs) {
        return this.add(rhs.inv());
    }

    public Vec2D mul(double k) {
        return new Vec2D(this.x * k, this.y * k);
    }

    public Vec2D div(double k) {
        return this.mul(1 / k);
    }

    public Vec2D rot(double phi) {
        return this.with_theta(this.theta() + phi);
    }

    public Vec2D max(Vec2D other) {
        if (this.r() > other.r()) {
            return this;
        }
        return other;
    }

    public Vec2D map_x(Function<Double, Double> mapping) {
        return this.with_x(mapping.apply(this.x));
    }

    public Vec2D map_y(Function<Double, Double> mapping) {
        return this.with_y(mapping.apply(this.y));
    }

    public static Vec2D from(Translation2d value) {
        return new Vec2D(value.getX(), value.getY());
    }

}
