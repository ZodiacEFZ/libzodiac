package frc.libzodiac.util;

import frc.libzodiac.Util;

import java.util.function.Function;

public class Vec2D {
    public final double x;
    public final double y;

    public Vec2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vec2D add(Vec2D a, Vec2D b) {
        return new Vec2D(a.x + b.x, a.y + b.y);
    }

    public static Polar add(Polar a, Polar b) {
        return Vec2D.add(a.to_vec(), b.to_vec()).to_polar();
    }

    public static double mul(Vec2D a, Vec2D b) {
        return a.r() * b.r() * Math.cos(a.theta() - b.theta());
    }

    public static double mul(Polar a, Polar b) {
        return Vec2D.mul(a.to_vec(), b.to_vec());
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
        return this.to_polar().with_r(r).to_vec();
    }

    public Vec2D with_theta(double theta) {
        return this.to_polar().with_theta(theta).to_vec();
    }

    public double r() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public double theta() {
        return Math.atan2(this.y, this.x);
    }

    public Vec2D inv() {
        return new Vec2D(-this.x, -this.y);
    }

    public Vec2D add(Vec2D rhs) {
        return Vec2D.add(this, rhs);
    }

    public Vec2D add(Polar rhs) {
        return this.add(rhs.to_vec());
    }

    public Vec2D sub(Vec2D rhs) {
        return Vec2D.add(this, rhs.inv());
    }

    public Vec2D sub(Polar rhs) {
        return this.add(rhs.to_vec().inv());
    }

    public Vec2D mul(double k) {
        return new Vec2D(this.x * k, this.y * k);
    }

    public Vec2D mul(Vec2D rhs) {
        return new Vec2D(this.x * rhs.x, this.y * rhs.y);
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

    public Polar to_polar() {
        return new Polar(this.r(), this.theta());
    }

    public record Polar(double r, double theta) {
        public Polar with_r(double r) {
            return new Polar(r, this.theta);
        }

        public Polar with_theta(double theta) {
            return new Polar(this.r, theta);
        }

        public Vec2D to_vec() {
            return new Vec2D(this.r * Math.cos(this.theta), this.r * Math.sin(this.theta));
        }

        public double x() {
            return this.to_vec().x;
        }

        public double y() {
            return this.to_vec().y;
        }

        public Polar add(Polar rhs) {
            return this.to_vec().add(rhs).to_polar();
        }

        public Polar add(Vec2D rhs) {
            return this.to_vec().add(rhs).to_polar();
        }

        public Polar sub(Polar rhs) {
            return this.to_vec().sub(rhs).to_polar();
        }

        public Polar sub(Vec2D rhs) {
            return this.to_vec().sub(rhs).to_polar();
        }

        public Polar mul(double k) {
            return this.to_vec().mul(k).to_polar();
        }

        public Polar div(double k) {
            return this.to_vec().div(k).to_polar();
        }

        public Polar rot(double phi) {
            return this.to_vec().rot(phi).to_polar();
        }
    }
}