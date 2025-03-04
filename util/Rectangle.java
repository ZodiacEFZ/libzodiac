package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class Rectangle {
    private final Translation2d bottomLeft;
    private final Translation2d bottomRight;
    private final Translation2d topLeft;
    private final Translation2d topRight;

    public Rectangle(Translation2d bottomLeft, Translation2d bottomRight, Translation2d topLeft,
            Translation2d topRight) {
        this.bottomLeft = bottomLeft;
        this.bottomRight = bottomRight;
        this.topLeft = topLeft;
        this.topRight = topRight;
    }

    public Rectangle(Translation2d bottomLeft, Translation2d topRight) {
        this.bottomLeft = bottomLeft;
        this.bottomRight = new Translation2d(topRight.getX(), bottomLeft.getY());
        this.topLeft = new Translation2d(bottomLeft.getX(), topRight.getY());
        this.topRight = topRight;
    }

    public Rectangle(Translation2d bottomLeft, Distance width, Distance height) {
        this(bottomLeft, width.in(Units.Meters), height.in(Units.Meters));
    }

    public Rectangle(Translation2d bottomLeft, double width, double height) {
        this.bottomLeft = bottomLeft;
        this.bottomRight = bottomLeft.plus(new Translation2d(width, 0));
        this.topLeft = bottomLeft.plus(new Translation2d(0, height));
        this.topRight = bottomLeft.plus(new Translation2d(width, height));
    }

    public Translation2d getBottomLeft() {
        return this.bottomLeft;
    }

    public Translation2d getBottomRight() {
        return this.bottomRight;
    }

    public Translation2d getTopLeft() {
        return this.topLeft;
    }

    public Translation2d getTopRight() {
        return this.topRight;
    }

    public double getWidth() {
        return this.topLeft.getDistance(this.topRight);
    }

    public double getHeight() {
        return this.topLeft.getDistance(this.bottomLeft);
    }

    public Rectangle translate(Translation2d translation) {
        return new Rectangle(this.bottomLeft.plus(translation), this.bottomRight.plus(translation),
                this.topLeft.plus(translation), this.topRight.plus(translation));
    }

    public Rectangle scale(double scale) {
        var center = this.getCenter();
        var newBottomLeft = this.bottomLeft.minus(center).times(scale).plus(center);
        var newBottomRight = this.bottomRight.minus(center).times(scale).plus(center);
        var newTopLeft = this.topLeft.minus(center).times(scale).plus(center);
        var newTopRight = this.topRight.minus(center).times(scale).plus(center);
        return new Rectangle(newBottomLeft, newBottomRight, newTopLeft, newTopRight);
    }

    public Translation2d getCenter() {
        return this.bottomLeft.plus(this.topRight).times(0.5);
    }

    public Rectangle rotateAroundOrigin(Rotation2d angle) {
        var newBottomLeft = this.bottomLeft.rotateBy(angle);
        var newBottomRight = this.bottomRight.rotateBy(angle);
        var newTopLeft = this.topLeft.rotateBy(angle);
        var newTopRight = this.topRight.rotateBy(angle);
        return new Rectangle(newBottomLeft, newBottomRight, newTopLeft, newTopRight);
    }

    public Rectangle rotateAroundCenter(Rotation2d angle) {
        return this.rotateAround(this.getCenter(), angle);
    }

    public Rectangle rotateAround(Translation2d other, Rotation2d angle) {
        var newBottomLeft = this.bottomLeft.rotateAround(other, angle);
        var newBottomRight = this.bottomRight.rotateAround(other, angle);
        var newTopLeft = this.topLeft.rotateAround(other, angle);
        var newTopRight = this.topRight.rotateAround(other, angle);
        return new Rectangle(newBottomLeft, newBottomRight, newTopLeft, newTopRight);
    }

    public boolean contains(Translation2d point) {
        return this.crossProduct(this.topLeft, this.bottomLeft, point) *
                       this.crossProduct(this.bottomRight, this.topRight, point) >= 0 &&
                       this.crossProduct(this.bottomLeft, this.bottomRight, point) *
                               this.crossProduct(this.topRight, this.topLeft, point) >= 0;
    }

    private double crossProduct(Translation2d a, Translation2d b, Translation2d c) {
        return this.crossProduct(a.minus(b), c.minus(b));
    }

    private double crossProduct(Translation2d a, Translation2d b) {
        return a.getX() * b.getY() - a.getY() * b.getX();
    }
}
