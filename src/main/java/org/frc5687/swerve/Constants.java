package org.frc5687.swerve;

import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.List;
import org.frc5687.swerve.subsystems.DiffSwerveModule;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;

public class Constants {
    public static final int TICKS_PER_UPDATE = 10;
    public static final double METRIC_FLUSH_PERIOD = 2.0;
    public static final double UPDATE_PERIOD = 0.02; // 20 ms
    public static final double CONTROL_PERIOD = 0.02; // 10 ms
    public static final double DATA_PERIOD = 0.02; // 20 ms
    public static final double EPSILON = 1e-9;

    /**
     * Coordinate System
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
    public static class DriveTrain {
        public static final String CAN_BUS = "CANivore";
        public static final int NUM_MODULES = 4;

        // Size of the robot chassis in meters
        public static final double WIDTH = 0.4445; // meters
        public static final double LENGTH = 0.4445; // meters
        // Distance of swerve modules from center of robot
        public static final double SWERVE_NS_POS = LENGTH / 2.0;
        public static final double SWERVE_WE_POS = WIDTH / 2.0;

        public static final double MAX_MPS = 4.2; // Max speed of robot (m/s)
        public static final double SLOW_MPS = 2.0; // Slow speed of robot (m/s)
        public static final double MAX_ANG_VEL = Math.PI; // Max rotation rate of robot (rads/s)
        public static final double SLOW_ANG_VEL = Math.PI; // Max rotation rate of robot (rads/s)

        public static final KinematicLimits KINEMATIC_LIMITS = new KinematicLimits();

        static {
            KINEMATIC_LIMITS.maxDriveVelocity = 5.3; // m/s
            KINEMATIC_LIMITS.maxDriveAcceleration = 25; // m/s^2
            KINEMATIC_LIMITS.maxSteeringVelocity = 25; // rad/s
        }
        public static final KinematicLimits DRIVE_POSE_KINEMATIC_LIMITS = new KinematicLimits();
        static {
            DRIVE_POSE_KINEMATIC_LIMITS.maxDriveVelocity = 2.5; // m/s
            DRIVE_POSE_KINEMATIC_LIMITS.maxDriveAcceleration = 20; // m/s^2
            DRIVE_POSE_KINEMATIC_LIMITS.maxSteeringVelocity = 20; // rad/s
        }

        public static final KinematicLimits TRAJECTORY_FOLLOWING = new KinematicLimits();
        static {
            TRAJECTORY_FOLLOWING.maxDriveVelocity = 5.0; // m/s
            TRAJECTORY_FOLLOWING.maxDriveAcceleration = 20; // m/s^2
            TRAJECTORY_FOLLOWING.maxSteeringVelocity = 20; // rad/s
        }
        public static final KinematicLimits SLOW_KINEMATIC_LIMITS = new KinematicLimits();

        static {
            SLOW_KINEMATIC_LIMITS.maxDriveVelocity = 2; // m/s
            SLOW_KINEMATIC_LIMITS.maxDriveAcceleration = 10; // m/s^2
            SLOW_KINEMATIC_LIMITS.maxSteeringVelocity = 10; // rad/s
        }

        public static final KinematicLimits VISION_KINEMATIC_LIMITS = new KinematicLimits();

        static {
            VISION_KINEMATIC_LIMITS.maxDriveVelocity = 3.0; // m/s
            VISION_KINEMATIC_LIMITS.maxDriveAcceleration = 15; // m/s^2
            VISION_KINEMATIC_LIMITS.maxSteeringVelocity = 20; // rad/s
        }

        public static final KinematicLimits POV_KINEMATIC_LIMITS = new KinematicLimits();

        static {
            POV_KINEMATIC_LIMITS.maxDriveVelocity = 1; // m/s
            POV_KINEMATIC_LIMITS.maxDriveAcceleration = 10; // m/s^2
            POV_KINEMATIC_LIMITS.maxSteeringVelocity = 10; // rad/s
        }

        public static final DiffSwerveModule.ModuleConfiguration NORTH_WEST_CONFIG =
                new DiffSwerveModule.ModuleConfiguration();

        static {
            NORTH_WEST_CONFIG.moduleName = "North West";
            NORTH_WEST_CONFIG.canBus = CAN_BUS;
            NORTH_WEST_CONFIG.position = new Translation2d(SWERVE_NS_POS, SWERVE_WE_POS); // +,+

            NORTH_WEST_CONFIG.encoderInverted = false;
            NORTH_WEST_CONFIG.encoderOffset = -0.07617;
        }

        public static final DiffSwerveModule.ModuleConfiguration SOUTH_WEST_CONFIG =
                new DiffSwerveModule.ModuleConfiguration();

        static {
            SOUTH_WEST_CONFIG.moduleName = "South West";
            SOUTH_WEST_CONFIG.canBus = CAN_BUS;
            SOUTH_WEST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, SWERVE_WE_POS); // -,+

            SOUTH_WEST_CONFIG.encoderInverted = false;
            SOUTH_WEST_CONFIG.encoderOffset = -0.1624;
        }

        public static final DiffSwerveModule.ModuleConfiguration SOUTH_EAST_CONFIG =
                new DiffSwerveModule.ModuleConfiguration();

        static {
            SOUTH_EAST_CONFIG.moduleName = "South East";
            SOUTH_EAST_CONFIG.canBus = CAN_BUS;
            SOUTH_EAST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, -SWERVE_WE_POS); // -,-

            SOUTH_EAST_CONFIG.encoderInverted = false;
            SOUTH_EAST_CONFIG.encoderOffset = -0.05523;
        }

        public static final DiffSwerveModule.ModuleConfiguration NORTH_EAST_CONFIG =
                new DiffSwerveModule.ModuleConfiguration();

        static {
            NORTH_EAST_CONFIG.moduleName = "North East";
            NORTH_EAST_CONFIG.canBus = CAN_BUS;
            NORTH_EAST_CONFIG.position = new Translation2d(SWERVE_NS_POS, -SWERVE_WE_POS); // +,-

            NORTH_EAST_CONFIG.encoderInverted = false;
            NORTH_EAST_CONFIG.encoderOffset = -0.0575;
        }

        public static final double TRANSLATION_DEADBAND = 0.05; // Avoid unintentional joystick movement
        public static final double ROTATION_DEADBAND = 0.05; // Avoid unintentional joystick movement
        public static final long DISABLE_TIME = 500; // ms

        public static final double LINEAR_VELOCITY_REFERENCE = 0.5;

        // Maximum rates of motion

        public static final double POLE_THRESHOLD = Units.degreesToRadians(5.0);

        // PID controller settings
        public static final double MAINTAIN_kP = 4.0;
        public static final double MAINTAIN_kI = 0.0;
        public static final double MAINTAIN_kD = 0.1;

        public static final double SNAP_kP = 4.0;
        public static final double SNAP_kI = 0.0;
        public static final double SNAP_kD = 0.1;

        public static final double SNAP_TOLERANCE = Units.degreesToRadians(5.0);

        public static final double PROFILE_CONSTRAINT_VEL = Math.PI * 4.0;
        public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI * 8.0;
 
        public static final double kP = 3.3;
        public static final double kI = 0.0;
        public static final double kD = 0.05;
        
        public static final double X_TRAJECTORY_kP = 3.8;
        public static final double X_TRAJECTORY_kI = 0.0;
        public static final double X_TRAJECTORY_kD = 0.02;
        
        public static final double Y_TRAJECTORY_kP = 3.8;
        public static final double Y_TRAJECTORY_kI = 0.0;
        public static final double Y_TRAJECTORY_kD = 0.02;

        public static final double ANGLE_TRAJECTORY_kP = 3.2;
        public static final double ANGLE_TRAJECTORY_kI = 0.0;
        public static final double ANGLE_TRAJECTORY_kD = 0.05;

        public static final double POSITION_TOLERANCE = 0.01;
        public static final double LEVEL_TOLERANCE = 0.5;
        public static final double HEADING_TOLERANCE = 0.15; // rad
        public static final double BUMP_DEGREES = 7;

        public static final double PITCH_LOOKING_ANGLE =
                Units.degreesToRadians(15.0); // this is degrees because sad.
        public static final double PITCH_LEVELED_ANGLE =
                Units.degreesToRadians(5.0); // this is degrees because sad.

        public static final double DRIVING_UP_RAMP_SPEEDS_VX = 2.0;
        public static final double DRIVING_DOWN_RAMP_SPEEDS_VX = 1.0;

        public static final double AUTO_LEVEL_KP = 4.5; //PID controller for leveling
        public static final double AUTO_LEVEL_KI = 0.0;
        public static final double AUTO_LEVEL_KD = 1.0;
        
        public static final double QUICK_LEVEL_KP = 3.0; //PID controller for leveling
        public static final double QUICK_LEVEL_KI = 0.0;
        public static final double QUICK_LEVEL_KD = 0.5;
    }


    public static class DifferentialSwerveModule {
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();
        // this is the motor config for the diff swerve motors
        static {
            CONFIG.TIME_OUT = 0.1;

            CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            CONFIG.INVERTED = InvertedValue.CounterClockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_STATOR_CURRENT = 120;
            CONFIG.MAX_CURRENT = 120;
            CONFIG.ENABLE_STATOR_CURRENT_LIMIT = true;
            CONFIG.CURRENT_DEADBAND = 0.1; // amps
//                        CONFIG.USE_FOC = true;
        }

        public static final OutliersTalon.ClosedLoopConfiguration CLOSED_LOOP_CONFIGURATION =
                new OutliersTalon.ClosedLoopConfiguration();

        //         update rate of our modules 5ms.
        public static final double kDt = 0.005;
        //        public static final double kDt = 0.01;
        public static final double FALCON_FREE_SPEED =
                Units.rotationsPerMinuteToRadiansPerSecond(6080); // was 6380 foc is different speed
        public static final double GEAR_RATIO_WHEEL = 6.46875 / 1.2;
        public static final double GEAR_RATIO_STEER = 9.2 / 1.2;

        public static final double FRICTION_STEER = 0.00;
        public static final double FRICTION_WHEEL = 0.00;
        public static final double WHEEL_RADIUS = 0.0457; // Meters with compression.
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double VOLTAGE = 12.0;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_STEER = 0.001;
        public static final double INERTIA_WHEEL = 0.001;
        // A weight for how aggressive each state should be ie. 0.08 radians will try to control the
        // angle more aggressively than the wheel angular velocity.

        public static final double Q_AZIMUTH = 0.06; // radians
        public static final double Q_AZIMUTH_ANG_VELOCITY = 2.0; // radians per sec
        public static final double Q_WHEEL_ANG_VELOCITY = 0.8; // radians per sec

        public static final double CONTROL_EFFORT = 4.0;
        // This is for Kalman filter which isn't used for azimuth angle due to angle wrapping.
        // Model noise are assuming that our model isn't as accurate as our sensors.
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = 0.1; // radians
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 1.0; // radians per sec
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 1.0; // radians per sec
        // Noise from sensors. Falcon With Gearbox causes us to have more uncertainty, so we
        // increase the noise.
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.01; // radians
        public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double MAX_MODULE_SPEED_MPS =
                (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * WHEEL_RADIUS;
        public static final double MAX_ANGULAR_VELOCITY = FALCON_FREE_SPEED / GEAR_RATIO_STEER;
        public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_VELOCITY * 5;

        public static final double MAX_MODULE_ACCELERATION = (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * 4;
        public static final double MAX_MODULE_JERK = MAX_MODULE_ACCELERATION * 2;
    }
}
