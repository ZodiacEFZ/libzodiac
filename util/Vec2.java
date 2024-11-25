package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Function;

/**
 * A utility for handling 2-dimentional vector calculations.
 */
public record Vec2(double x, double y) {

    /**
     * Conversion form WPILib's <code>Translation2d</code>.
     *
     * @param value value to convert
     * @return converted new instance
     */
    public static Vec2 from(Translation2d value) {
        return new Vec2(value.getX(), value.getY());
    }

    /**
     * Get x component of the vector.
     *
     * @return x
     */
    public double x() {
        return this.x;
    }

    /**
     * Get y component of the vector.
     *
     * @return y
     */
    public double y() {
        return this.y;
    }

    /**
     *
     */
    @Override
    public String toString() {
        return "(" + Util.printed(this.x) + "," + Util.printed(this.y) + ")";
    }

    /**
     * The substraction operation.
     *
     * @param rhs right-hand side of operator
     * @return result of operator
     */
    public Vec2 sub(Vec2 rhs) {
        return this.add(rhs.inv());
    }

    /**
     * The addition operation.
     *
     * @param rhs right-hand side of operator
     * @return result of operator
     */
    public Vec2 add(Vec2 rhs) {
        return new Vec2(this.x + rhs.x, this.y + rhs.y);
    }

    /**
     * Invert the vector to opposite direction.
     *
     * @return an inverted vector
     */
    public Vec2 inv() {
        return new Vec2(-this.x, -this.y);
    }

    /**
     * The dot-product operation.
     *
     * @param rhs right-hand side of operator
     * @return result of operator
     */
    public double mul(Vec2 rhs) {
        return this.x * rhs.x + this.y * rhs.y;
    }

    /**
     * The division operation.
     *
     * @param k the divisor
     * @return result of operator
     */
    public Vec2 div(double k) {
        return this.mul(1 / k);
    }

    /**
     * The multiplication operation.
     *
     * @param k the multiplier
     * @return result of operator
     */
    public Vec2 mul(double k) {
        return new Vec2(this.x * k, this.y * k);
    }

    /**
     * Rotate the vector a specific angle. CCW positive.
     *
     * @param phi the angle to rotate
     * @return a rotated vector
     */
    public Vec2 rot(double phi) {
        return this.with_theta(this.theta() + phi);
    }

    /**
     * Create new vector instance with specified direction, preserving original
     * radius/magnitude.
     *
     * @param theta the new direction
     * @return the new vector
     */
    public Vec2 with_theta(double theta) {
        return polar(this.r(), theta);
    }

    /**
     * Get the direction of the vector.
     *
     * @return in [-pi,pi)
     */
    public double theta() {
        return Math.atan2(this.y, this.x);
    }

    /**
     * Construct a vector from polar coordinates, i.e. (r,theta) -> (x,y).
     *
     * @param r     the radius of polar coordinates
     * @param theta the angle of polar coordinates
     * @return a rectangular vector
     */
    public static Vec2 polar(double r, double theta) {
        return new Vec2(r * Math.cos(theta), r * Math.sin(theta));
    }

    /**
     * Get the radius/magnitude of the vector.
     *
     * @return positive
     */
    public double r() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    /**
     * Calculate the vector's projection on another vector.
     *
     * @param on the vector to project on
     * @return the projected vector
     */
    public Vec2 proj(Vec2 on) {
        return on.with_r(this.r() * Math.cos(this.theta() - on.theta()));
    }

    /**
     * Create new vector instance with specified radius, preserving
     * original direction.
     *
     * @param r the new radius
     * @return the new vector
     */
    public Vec2 with_r(double r) {
        return polar(r, this.theta());
    }

    /**
     * Compare the magnitude of two vectors, return the maximum.
     *
     * @param other another vector
     * @return the larger one
     */
    public Vec2 max(Vec2 other) {
        return this.r() > other.r() ? this : other;
    }

    /**
     * Compare the magnitude of two vectors, return the minimum.
     *
     * @param other another vector
     * @return the smaller one
     */
    public Vec2 min(Vec2 other) {
        return this.r() < other.r() ? this : other;
    }

    /**
     * Map the x component of the vector.
     *
     * @param mapping mapping to apply to x
     * @return a mapped vector
     */
    public Vec2 map_x(Function<Double, Double> mapping) {
        return this.with_x(mapping.apply(this.x));
    }

    /**
     * Create new vector instance with specified x component, preserving original y
     * component.
     *
     * @param x the new x component
     * @return the new vector
     */
    public Vec2 with_x(double x) {
        return new Vec2(x, this.y);
    }

    /**
     * Map the y component of the vector.
     *
     * @param mapping mapping to apply to y
     * @return a mapped vector
     */
    public Vec2 map_y(Function<Double, Double> mapping) {
        return this.with_y(mapping.apply(this.y));
    }

    /**
     * Create new vector instance with specified y component, preserving original x
     * component.
     *
     * @param y the new y component
     * @return the new vector
     */
    public Vec2 with_y(double y) {
        return new Vec2(this.x, y);
    }

}
