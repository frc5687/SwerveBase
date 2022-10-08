/* Team 5687 (C)2022 */
package org.frc5687.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5687.swerve.Constants;
import org.frc5687.swerve.util.Helpers;

public class Vector2d {
    protected static final Vector2d IDENTITY = new Vector2d();

    public static Vector2d identity() {
        return IDENTITY;
    }
    protected double _x;
    protected double _y;

    public Vector2d() {
        _x = 0;
        _y = 0;
    }

    public Vector2d(double x, double y) {
        _x = x;
        _y = y;
    }

    public Vector2d(Vector2d other) {
        _x = other._x;
        _y = other._y;
    }

    public void setX(double x) {
        _x = x;
    }

    public double x() {
        return _x;
    }

    public void setY(double y) {
        _y = y;
    }

    public double y() {
        return _y;
    }

    public double magnitude() {
        return Math.hypot(_x, _y);
    }

    public Vector2d normalize() {
        if (equals(new Vector2d())) return this;
        return scale(1.0 / magnitude());
    }

    public Rotation2d direction() {
        return new Rotation2d(_x, _y);
    }

    public Vector2d scale(double scalar) {
        return new Vector2d(_x * scalar, _y * scalar);
    }

    public Vector2d plus(Vector2d other) {
        return new Vector2d(_x + other._x, _y + other._y);
    }

    public Vector2d minus(Vector2d other) {
        return new Vector2d(_x - other._x, _y - other._y);
    }

    public double dot(Vector2d other) {
        return _x * other._x + _y * other._y;
    }

    public Rotation2d angle(Vector2d other) {
        double cosAngle = dot(other) / (magnitude() * other.magnitude());
        if (Double.isNaN(cosAngle)) {
            return new Rotation2d();
        }
        return new Rotation2d(Math.acos(Math.min(1.0, Math.max(cosAngle, -1.0))));
    }

    public boolean equals(Vector2d other) {
        return Helpers.epsilonEquals(_x, other._x, Constants.EPSILON)
                && Helpers.epsilonEquals(_y, other._y, Constants.EPSILON);
    }
}
