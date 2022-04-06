/* Team 5687 (C)2020-2022 */
package org.frc5687.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final double UPDATE_PERIOD = 0.02;
    public static final double EPSILON = 0.00001;

    public static class DriveTrain {
        public static final String CAN_BUS = "rio";
        public static final double kDt = 0.02;
        public static final double TRANSLATION_DEADBAND =
                0.1; // Avoid unintentional joystick movement
        public static final double ROTATION_DEADBAND = 0.1; // Avoid unintentional joystick movement
        public static final double POWER = 1.75; // Determines the curve of drive input

        // Size of the robot chassis in meters
        public static final double WIDTH = 0.6223; // meters
        public static final double LENGTH = 0.6223; // meters

        // Distance of swerve modules from center of robot
        public static final double SWERVE_NS_POS = LENGTH / 2.0;

        public static final double SWERVE_WE_POS = WIDTH / 2.0;

        /**
         * Coordinate system is wacky:
         *
         * <p>(X, Y): X is N or S, N is + Y is W or E, W is +
         *
         * <p>NW (+,+) NE (+,-)
         *
         * <p>SW (-,+) SE (-,-)
         *
         * <p>We go counter-counter clockwise starting at NW of chassis:
         *
         * <p>NW, SW, SE, NE
         *
         * <p>Note: when robot is flipped over, this is clockwise.
         */

        // Position vectors for the swerve module kinematics
        // i.e. location of each swerve module from center of robot
        // see coordinate system above to understand signs of vector coordinates
        public static final Translation2d NORTH_WEST =
                new Translation2d(SWERVE_NS_POS, SWERVE_WE_POS); // +,+

        public static final Translation2d SOUTH_WEST =
                new Translation2d(-SWERVE_NS_POS, SWERVE_WE_POS); // -,+
        public static final Translation2d SOUTH_EAST =
                new Translation2d(-SWERVE_NS_POS, -SWERVE_WE_POS); // -,-
        public static final Translation2d NORTH_EAST =
                new Translation2d(SWERVE_NS_POS, -SWERVE_WE_POS); // +,-

        // Should be 0, but can correct for hardware error in swerve module headings here.
        public static final double NORTH_WEST_OFFSET = 0; // radians
        public static final double SOUTH_WEST_OFFSET = 0; // radians
        public static final double SOUTH_EAST_OFFSET = 0; // radians
        public static final double NORTH_EAST_OFFSET = 0; // radians

        // In case encoder is measuring rotation in the opposite direction we expect.
        public static final boolean NORTH_WEST_ENCODER_INVERTED = false;
        public static final boolean SOUTH_WEST_ENCODER_INVERTED = false;
        public static final boolean SOUTH_EAST_ENCODER_INVERTED = false;
        public static final boolean NORTH_EAST_ENCODER_INVERTED = false;

        // Maximum rates of motion
        public static final double MAX_MPS = 3.0; // Max speed of robot (m/s)
        public static final double MAX_MPS_DURING_CLIMB =
                MAX_MPS / 4; // Max speed of robot (m/s) during climb
        public static final double MAX_ANG_VEL =
                Math.PI * 1.5; // Max rotation rate of robot (rads/s)
        public static final double MAX_MPSS = 0.5; // Max acceleration of robot (m/s^2)

        public static final double POLE_THRESHOLD = Units.degreesToRadians(5.0);

        // PID controller settings
        public static final double STABILIZATION_kP = 2.0;
        public static final double STABILIZATION_kI = 0.0;
        public static final double STABILIZATION_kD = 0.0;

        public static final double SNAP_kP = 4.0;
        public static final double SNAP_kI = 0.0;
        public static final double SNAP_kD = 0.0;

        public static final double VISION_kP = 4.0;
        public static final double VISION_kI = 0.0;
        public static final double VISION_kD = 0.0;

        public static final double PROFILE_CONSTRAINT_VEL = MAX_ANG_VEL;
        public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI * 3.0;

        public static final double kP = 11.5;
        public static final double kI = 0.0;
        public static final double kD = 0.5;
    }

    public static class DifferentialSwerveModule {

        // update rate of our modules 5ms.
        public static final double kDt = 0.005;
        public static final int FALCON_VELOCITY_MEASUREMENT_WINDOW = 32;
        public static final int TIMEOUT = 10;

        public static final double FALCON_FREE_SPEED =
                Units.rotationsPerMinuteToRadiansPerSecond(6380);
        public static final double GEAR_RATIO_WHEEL = 6.46875;
        public static final double GEAR_RATIO_STEER = 9.2;
        public static final double FALCON_RATE = 600.0;
        public static final double WHEEL_RADIUS = 0.04615; // Meters with compression.
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double VOLTAGE = 12.0;
        public static final double FEED_FORWARD = VOLTAGE / (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL);

        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final double CURRENT_LIMIT = 30.0;
        public static final double CURRENT_THRESHOLD = 30.0;
        public static final double CURRENT_TRIGGER_TIME = 0.0;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_WHEEL = 0.005;
        public static final double INERTIA_STEER = 0.004;
        // A weight for how aggressive each state should be ie. 0.08 radians will try to control the
        // angle more aggressively than the wheel angular velocity.
        public static final double Q_AZIMUTH_ANG_VELOCITY = 1.1; // radians per sec
        public static final double Q_AZIMUTH = 0.08; // radians
        public static final double Q_WHEEL_ANG_VELOCITY = 5; // radians per sec
        // This is for Kalman filter which isn't used for azimuth angle due to angle wrapping.
        // Model noise are assuming that our model isn't as accurate as our sensors.
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = .1; // radians
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        // Noise from sensors. Falcon With Gearbox causes us to have more uncertainty, so we
        // increase the noise.
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.01; // radians
        public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double CONTROL_EFFORT = VOLTAGE;

        public static final double MAX_MODULE_SPEED_MPS =
                (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * WHEEL_RADIUS;
        public static final double MAX_ANGULAR_VELOCITY = FALCON_FREE_SPEED / GEAR_RATIO_STEER;
        public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_VELOCITY * 10;

        public static final double MAX_MODULE_ACCELERATION =
                (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * 1.5;
        public static final double MAX_MODULE_JERK = MAX_MODULE_ACCELERATION * 10;
    }
}
