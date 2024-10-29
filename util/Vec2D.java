package frc.libzodiac.util;

import frc.libzodiac.Util;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A utility for handling 2-dimentional vector calculations.
 */
public record Vec2D(double x, double y) {

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
     * Get the radius/magnitude of the vector.
     * 
     * @return positive
     */
    public double r() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
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
    public static Vec2D polar(double r, double theta) {
        return new Vec2D(r * Math.cos(theta), r * Math.sin(theta));
    }

    /**
     * 
     */
    @Override
    public String toString() {
        return "(" + Util.printed(this.x) + "," + Util.printed(this.y) + ")";
    }

    /**
     * Create new vector instance with specified x component, preserving original y
     * component.
     * 
     * @param x the new x component
     * @return the new vector
     */
    public Vec2D with_x(double x) {
        return new Vec2D(x, this.y);
    }

    /**
     * Create new vector instance with specified y component, preserving original x
     * component.
     * 
     * @param y the new y component
     * @return the new vector
     */
    public Vec2D with_y(double y) {
        return new Vec2D(this.x, y);
    }

    /**
     * Create new vector instance with specified radius, preserving
     * original direction.
     * 
     * @param r the new radius
     * @return the new vector
     */
    public Vec2D with_r(double r) {
        return polar(r, this.theta());
    }

    /**
     * Create new vector instance with specified direction, preserving original
     * radius/magnitude.
     * 
     * @param theta the new direction
     * @return the new vector
     */
    public Vec2D with_theta(double theta) {
        return polar(this.r(), theta);
    }

    /**
     * Invert the vector to opposite direction.
     * 
     * @return an inverted vector
     */
    public Vec2D inv() {
        return new Vec2D(-this.x, -this.y);
    }

    /**
     * The addition operation.
     * 
     * @param rhs right-hand side of operator
     * @return result of operator
     */
    public Vec2D add(Vec2D rhs) {
        return new Vec2D(this.x + rhs.x, this.y + rhs.y);
    }

    /**
     * The substraction operation.
     * 
     * @param rhs right-hand side of operator
     * @return result of operator
     */
    public Vec2D sub(Vec2D rhs) {
        return this.add(rhs.inv());
    }

    /**
     * The multiplication operation.
     * 
     * @param k the multiplier
     * @return result of operator
     */
    public Vec2D mul(double k) {
        return new Vec2D(this.x * k, this.y * k);
    }

    /**
     * The division operation.
     * 
     * @param k the divisor
     * @return result of operator
     */
    public Vec2D div(double k) {
        return this.mul(1 / k);
    }

    /**
     * Rotate the vector a specific angle. CCW positive.
     * 
     * @param phi the angle to rotate
     * @return a rotated vector
     */
    public Vec2D rot(double phi) {
        return this.with_theta(this.theta() + phi);
    }

    /**
     * Compare the magnitude of two vectors, return the maximum.
     * 
     * @param other another vector
     * @return the larger one
     */
    public Vec2D max(Vec2D other) {
        return this.r() > other.r() ? this : other;
    }

    /**
     * Compare the magnitude of two vectors, return the minimum.
     * 
     * @param other another vector
     * @return the smaller one
     */
    public Vec2D min(Vec2D other) {
        return this.r() < other.r() ? this : other;
    }

    /**
     * Map the x component of the vector.
     * 
     * @param mapping mapping to apply to x
     * @return a mapped vector
     */
    public Vec2D map_x(Function<Double, Double> mapping) {
        return this.with_x(mapping.apply(this.x));
    }

    /**
     * Map the y component of the vector.
     * 
     * @param mapping mapping to apply to y
     * @return a mapped vector
     */
    public Vec2D map_y(Function<Double, Double> mapping) {
        return this.with_y(mapping.apply(this.y));
    }

    /**
     * Conversion form WPILib's <code>Translation2d</code>.
     * 
     * @param value value to convert
     * @return converted new instance
     */
    public static Vec2D from(Translation2d value) {
        return new Vec2D(value.getX(), value.getY());
    }

}
