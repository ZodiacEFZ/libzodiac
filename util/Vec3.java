package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * A utility for handling 3-dimentional vector calculations.
 */
public record Vec3(double x, double y, double z) {

    /**
     * Conversion form WPILib's <code>Translation3d</code>.
     *
     * @param value value to convert
     * @return converted new instance
     */
    public static Vec3 from(Translation3d value) {
        return new Vec3(value.getX(), value.getY(), value.getZ());
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
     * Get z component of the vector.
     * 
     * @return z
     */
    public double z() {
        return this.z;
    }

    /**
     * Get the radius/magnitude of the vector.
     * 
     * @return positive
     */
    public double r() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    /**
     * Get the angle of the vector on xy-plane.
     * 
     * @return in [-pi,pi)
     */
    public double theta() {
        return Math.atan2(this.y, this.x);
    }

    /**
     * Get the angle of the vector to z-axis (zenith angle).
     * 
     * @return in [-pi/2,pi/2]
     */
    public double phi() {
        return Math.acos(this.z / this.r());
    }

    /**
     * Construct a vector from polar coordinates, i.e. (r,theta,phi) -> (x,y,z).
     * 
     * @param r     the radius of polar coordinates
     * @param theta the angle on xy-plane
     * @param phi   the angle to xy-plane
     * @return a rectangular vector
     */
    public static Vec3 polar(double r, double theta, double phi) {
        final var z = r * Math.cos(phi);
        final var xy = r * Math.sin(phi);
        final var x = xy * Math.cos(theta);
        final var y = xy * Math.sin(theta);
        return new Vec3(x, y, z);
    }

    @Override
    public String toString() {
        return "(" + Util.printed(this.x) + "," + Util.printed(this.y) + "," + Util.printed(this.z) + ")";
    }

    /**
     * The addition operation.
     * 
     * @param rhs right-hand side of operator
     * @return result of operator
     */
    public Vec3 add(Vec3 rhs) {
        return new Vec3(this.x + rhs.x, this.y + rhs.y, this.z + rhs.z);
    }

    /**
     * Invert the vector to opposite direction.
     * 
     * @return an inverted vector
     */
    public Vec3 inv() {
        return new Vec3(-this.x, -this.y, -this.z);
    }

    /**
     * The substraction operation.
     * 
     * @param rhs right-hand side of operator
     * @return result of operator
     */
    public Vec3 sub(Vec3 rhs) {
        return this.add(rhs.inv());
    }

    /**
     * The multiplication (dot product) operation.
     * 
     * @param k the multiplier
     * @return result of operator
     */
    public Vec3 mul(double k) {
        return new Vec3(this.x * k, this.y * k, this.z * k);
    }

    /**
     * The division operation.
     * 
     * @param k the divisor
     * @return result of operator
     */
    public Vec3 div(double k) {
        return this.mul(1 / k);
    }

    /**
     * The multiplication (dot product) operation.
     * 
     * @param rhs right-hand side of operator
     * @return result of operator
     */
    public double dot(Vec3 rhs) {
        return this.x * rhs.x + this.y * rhs.y + this.z * rhs.z;
    }

    /**
     * The multiplication (cross product) operation.
     * 
     * @param rhs right-hand side of operator
     * @return result of operator
     */
    public Vec3 cross(Vec3 rhs) {
        final var x = this.y * rhs.z - this.z * rhs.y;
        final var y = this.z * rhs.x - this.x * rhs.z;
        final var z = this.x * rhs.y - this.y * rhs.z;
        return new Vec3(x, y, z);
    }

    /**
     * Calculate the vector's projection on another vector.
     *
     * @param on the vector to project on
     * @return the projected vector
     */
    public Vec3 proj(Vec3 on) {
        return on.mul(this.dot(on) / (on.r() * on.r()));
    }

    /**
     * Create new vector instance with specified x component, preserving original y
     * and z component.
     * 
     * @param x the new x component
     * @return the new vector
     */
    public Vec3 with_x(double x) {
        return new Vec3(x, this.y, this.z);
    }

    /**
     * Create new vector instance with specified y component, preserving original x
     * and z component.
     * 
     * @param y the new y component
     * @return the new vector
     */
    public Vec3 with_y(double y) {
        return new Vec3(this.x, y, this.z);
    }

    /**
     * Create new vector instance with specified z component, preserving original x
     * and y component.
     * 
     * @param z the new z component
     * @return the new vector
     */
    public Vec3 with_z(double z) {
        return new Vec3(this.x, this.y, z);
    }

    /**
     * Create new vector instance with specified radius, preserving
     * original direction.
     * 
     * @param r the new radius
     * @return the new vector
     */
    public Vec3 with_r(double r) {
        return polar(r, this.theta(), this.phi());
    }

    /**
     * Create new vector instance with specified angle on xy-plane, preserving
     * original radius/magnitude.
     * 
     * @param theta the new angle on xy-plane
     * @return the new vector
     */
    public Vec3 with_theta(double theta) {
        return polar(this.r(), theta, this.phi());
    }

    /**
     * Create new vector instance with specified angle to z-axis (zenith angle),
     * preserving original radius/magnitude.
     * 
     * @param theta the new angle to xy-plane
     * @return the new vector
     */
    public Vec3 with_phi(double phi) {
        return polar(this.r(), this.theta(), phi);
    }

}
