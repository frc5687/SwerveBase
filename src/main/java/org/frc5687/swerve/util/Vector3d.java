/* Team 5687 (C)2022 */
package org.frc5687.swerve.util;

public class Vector3d {
    protected double _x;
    protected double _y;
    protected double _z;

    public Vector3d() {
        _x = 0;
        _y = 0;
        _z = 0;
    }

    public Vector3d(double x, double y, double z) {
        _x = x;
        _y = y;
        _z = z;
    }

    public Vector3d(Vector3d other) {
        _x = other._x;
        _y = other._y;
        _z = other._z;
    }

    public double magnitude() {
        return Math.sqrt((_x * _x) + (_y * _y) + (_z * _z));
    }
}
