/* Team 5687 (C)2022 */
package org.frc5687.swerve.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtil {

    public static Rotation2d getNearestPole(Rotation2d rot) {
        double pole_sin;
        double pole_cos;
        if (Math.abs(rot.getCos()) > Math.abs(rot.getSin())) {
            pole_cos = Math.signum(rot.getCos());
            pole_sin = 0.0;
        } else {
            pole_cos = 0.0;
            pole_sin = Math.signum(rot.getSin());
        }
        return new Rotation2d(pole_cos, pole_sin);
    }

    public static Rotation2d getDirection(Translation2d translation) {
        return new Rotation2d(translation.getX(), translation.getY());
    }

    public static Rotation2d inverse(Rotation2d rot) {
        return new Rotation2d(rot.getCos(), -rot.getSin());
    }

    public static double getDistance(Rotation2d rot, Rotation2d other) {
        return inverse(rot).rotateBy(other).getRadians();
    }

    public static Translation2d rotationToTranslation(Rotation2d rot) {
        return new Translation2d(rot.getCos(), rot.getSin());
    }
}
