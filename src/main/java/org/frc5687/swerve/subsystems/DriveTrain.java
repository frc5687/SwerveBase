/* Team 5687 (C)2020-2022 */
package org.frc5687.swerve.subsystems;

import static org.frc5687.swerve.Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS;
import static org.frc5687.swerve.Constants.DriveTrain.*;
import static org.frc5687.swerve.util.GeometryUtil.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.List;
import org.frc5687.swerve.Constants;
import org.frc5687.swerve.RobotMap;
import org.frc5687.swerve.util.*;

public class DriveTrain extends OutliersSubsystem {
    // Order we define swerve modules in kinematics
    // NB: must be same order as we pass to SwerveDriveKinematics
    private DiffSwerveModule _northWest, _southWest, _northEast, _southEast;
    private List<DiffSwerveModule> _modules;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odometry;

    // teleop values
    private Vector2d _translationVector;
    private double _rotationInput;
    private ControlState _controlState;
    private boolean _fieldRelative;
    private boolean _lockHeading;

    private AHRS _imu;
    private HolonomicDriveController _poseController;
    private SwerveHeadingController _headingController;

    // Trajectory / Pose Following
    private Trajectory.State _trajectoryGoal;
    private Rotation2d _trajectoryHeading;
    private Pose2d _goalPose;

    private double _driveSpeed = Constants.DriveTrain.MAX_MPS;

    private boolean _isMoving = false;
    private boolean _climbing = false;

    public DriveTrain(OutliersContainer container, AHRS imu) {
        super(container);
        try {
            _imu = imu;
            _northWest =
                    new DiffSwerveModule(
                            Constants.DriveTrain.NORTH_WEST,
                            RobotMap.CAN.TALONFX.NORTH_WEST_OUTER,
                            RobotMap.CAN.TALONFX.NORTH_WEST_INNER,
                            RobotMap.DIO.NORTH_WEST,
                            Constants.DriveTrain.NORTH_WEST_OFFSET,
                            Constants.DriveTrain.NORTH_WEST_ENCODER_INVERTED,
                            Constants.DriveTrain.CAN_BUS);
            _southWest =
                    new DiffSwerveModule(
                            Constants.DriveTrain.SOUTH_WEST,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_OUTER,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_INNER,
                            RobotMap.DIO.SOUTH_WEST,
                            Constants.DriveTrain.SOUTH_WEST_OFFSET,
                            Constants.DriveTrain.SOUTH_WEST_ENCODER_INVERTED,
                            CAN_BUS);
            _southEast =
                    new DiffSwerveModule(
                            Constants.DriveTrain.SOUTH_EAST,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_INNER,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_OUTER,
                            RobotMap.DIO.SOUTH_EAST,
                            Constants.DriveTrain.SOUTH_EAST_OFFSET,
                            Constants.DriveTrain.SOUTH_EAST_ENCODER_INVERTED,
                            CAN_BUS);
            _northEast =
                    new DiffSwerveModule(
                            Constants.DriveTrain.NORTH_EAST,
                            RobotMap.CAN.TALONFX.NORTH_EAST_INNER,
                            RobotMap.CAN.TALONFX.NORTH_EAST_OUTER,
                            RobotMap.DIO.NORTH_EAST,
                            Constants.DriveTrain.NORTH_EAST_OFFSET,
                            Constants.DriveTrain.NORTH_EAST_ENCODER_INVERTED,
                            CAN_BUS);

            _modules = Arrays.asList(_northWest, _southWest, _southEast, _northEast);

            // NB: it matters which order these are defined
            _kinematics =
                    new SwerveDriveKinematics(
                            _northWest.getModulePosition(),
                            _southWest.getModulePosition(),
                            _southEast.getModulePosition(),
                            _northEast.getModulePosition());
            _odometry = new SwerveDriveOdometry(_kinematics, getHeading());

            _poseController =
                    new HolonomicDriveController(
                            new PIDController(
                                    Constants.DriveTrain.kP,
                                    Constants.DriveTrain.kI,
                                    Constants.DriveTrain.kD),
                            new PIDController(
                                    Constants.DriveTrain.kP,
                                    Constants.DriveTrain.kI,
                                    Constants.DriveTrain.kD),
                            new ProfiledPIDController(
                                    STABILIZATION_kP,
                                    STABILIZATION_kI,
                                    STABILIZATION_kD,
                                    new TrapezoidProfile.Constraints(
                                            Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
                                            Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)));

            _headingController = new SwerveHeadingController(Constants.DriveTrain.kDt);
            _translationVector = new Vector2d();
            _rotationInput = 0;
            _controlState = ControlState.NEUTRAL;
            _fieldRelative = true;
            _isMoving = false;
            _lockHeading = false;
            _trajectoryGoal = new Trajectory.State();
            _trajectoryHeading = new Rotation2d();
            _goalPose = new Pose2d();
        } catch (Exception e) {
            error(e.getMessage());
        }
    }

    // use for modules as controller is running at 200Hz.
    public void modulePeriodic() {
        _modules.forEach(DiffSwerveModule::periodic);
    }

    @Override
    public void controlPeriodic(double timestamp) {
        modulePeriodic();
        double omegaCorrection = _headingController.getRotationCorrection(getHeading());
        switch (_controlState) {
            case NEUTRAL:
                break;
            case MANUAL:
                updateSwerve(_translationVector, _rotationInput + omegaCorrection);
                break;
            case POSITION:
                updateSwerve(_goalPose);
                break;
            case ROTATION:
                updateSwerve(Vector2d.identity(), omegaCorrection);
                break;
            case TRAJECTORY:
                //                updateSwerve(_trajectoryGoal, _trajectoryHeading);
                break;
        }
    }

    @Override
    public void dataPeriodic(double timestamp) {
        updateOdometry();
    }

    public void startModules() {
        _modules.forEach(DiffSwerveModule::start);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int module = 0; module < _modules.size(); module++) {
            _modules.get(module).setIdealState(states[module]);
        }
    }

    public void setControlState(ControlState state) {
        _controlState = state;
    }

    public ControlState getControlState() {
        return _controlState;
    }

    /**
     * Method to set correct module speeds and angle based on wanted vx, vy, omega
     *
     * @param vx velocity in x direction
     * @param vy velocity in y direction
     * @param omega angular velocity (rotating speed)
     */
    public void drive(double vx, double vy, double omega) {
        if (_controlState == ControlState.NEUTRAL) {
            setControlState(ControlState.MANUAL);
        }
        Vector2d translation = new Vector2d(vx, vy);
        double magnitude = translation.magnitude();

        if (Math.abs(getDistance(translation.direction(), getNearestPole(translation.direction())))
                < POLE_THRESHOLD) {
            translation =
                    rotationToVector(getNearestPole(translation.direction())).scale(magnitude);
        }

        if (magnitude < TRANSLATION_DEADBAND) {
            translation = new Vector2d();
            magnitude = 0;
        }

        Rotation2d direction = translation.direction();
        double scaledMagnitude = Math.pow(magnitude, TRANSLATION_POWER);
        translation =
                new Vector2d(
                        direction.getCos() * scaledMagnitude, direction.getSin() * scaledMagnitude);

        if (translation.magnitude() > 1.0) {
            translation = translation.normalize();
        }

        omega = (Math.abs(omega) < ROTATION_DEADBAND) ? 0 : omega;
        // scale rotation
        omega = Math.pow(Math.abs(omega), ROTATION_POWER) * Math.signum(omega);

        translation = translation.scale(_driveSpeed);
        omega *= MAX_ANG_VEL;

        //        if (omega != 0 && _rotationInput == 0) {
        //            _headingController.disable();
        if (omega == 0) {
            if (!_lockHeading) {
                _headingController.temporaryDisable();
            }
            _lockHeading = true;
        } else {
            _headingController.disable();
            _lockHeading = false;
        }
        metric("target heading", _headingController.getTargetHeading().getRadians());

        _rotationInput = omega;
        _translationVector = translation;
        //        _isMoving = vx != 0 || vy != 0 || !(Math.abs(omega) < ROTATING_TOLERANCE);
    }

    public void updateSwerve(Vector2d translationVector, double rotationalInput) {
        SwerveModuleState[] swerveModuleStates =
                _kinematics.toSwerveModuleStates(
                        _fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translationVector.x(),
                                translationVector.y(),
                                rotationalInput,
                                getHeading())
                                : new ChassisSpeeds(
                                translationVector.x(),
                                translationVector.y(),
                                rotationalInput));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(swerveModuleStates);
    }

    public void updateSwerve(Trajectory.State goal, Rotation2d heading) {
        ChassisSpeeds adjustedSpeeds =
                _poseController.calculate(getOdometryPose(), goal, getHeading());
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(moduleStates);
    }

    public void updateSwerve(Pose2d pose) {
        ChassisSpeeds adjustedSpeeds =
                _poseController.calculate(
                        getOdometryPose(), pose, LINEAR_VELOCITY_REFERENCE, pose.getRotation());
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(moduleStates);
    }

    public double getYaw() {
        return _imu.getYaw();
    }

    // yaw is negative to follow wpi coordinate system.
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-getYaw());
    }

    public void resetYaw() {
        _imu.reset();
        _headingController.setStabilizationHeading(new Rotation2d(0.0));
    }

    public void snap(Rotation2d heading) {
        _headingController.setStabilizationHeading(heading);
    }

    public void stabilize(Rotation2d heading) {
        _headingController.setStabilizationHeading(heading);
    }

    public void vision(Rotation2d visionHeading) {
        _headingController.setVisionHeading(visionHeading);
    }

    public void disableHeadingController() {
        _headingController.setState(SwerveHeadingController.HeadingState.OFF);
    }

    public void rotate(Rotation2d heading) {
        if (_translationVector.equals(Vector2d.identity())) {
            rotateInPlace(heading);
        } else {
            _headingController.setStabilizationHeading(heading);
        }
    }

    public void rotateInPlace(Rotation2d heading) {
        setControlState(ControlState.ROTATION);
        _headingController.setSnapHeading(heading);
    }

    public Rotation2d getVisionHeading() {
        // create per new robot.
        return new Rotation2d();
    }

    public SwerveHeadingController.HeadingState getCurrentHeadingState() {
        return _headingController.getHeadingState();
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(
                Constants.DriveTrain.MAX_AUTO_MPS, Constants.DriveTrain.MAX_MPSS)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    public void setTrajectoryGoal(Trajectory.State goal, Rotation2d heading) {
        _trajectoryGoal = goal;
        _trajectoryHeading = heading;
    }

    public void setPoseGoal(Pose2d pose) {
        _goalPose = pose;
    }

    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, Constants.DriveTrain.MAX_AUTO_MPS);
    }

    /**
     * calculates time to hit moving target
     *
     * @param speed exit velocity
     * @return lead time
     */
    public double calculateLeadTime(Vector3d position, Vector3d velocity, double speed) {
        double c0 = position.dot(position);
        double c1 = position.dot(velocity);
        double c2 = (speed * speed) * velocity.dot(velocity);
        double calculation = c1 * c1 + c2 * c0;
        double time = 0;
        if (calculation >= 0) {
            time = (c1 + Math.sqrt(calculation)) / c0;
            if (time < 0) {
                time = 0;
            }
        }
        return time;
    }


    public boolean isAtPose(Pose2d pose) {
        double diffX = getOdometryPose().getX() - pose.getX();
        double diffY = getOdometryPose().getY() - pose.getY();
        return (Math.abs(diffX) <= Constants.DriveTrain.POSITION_TOLERANCE)
                && (Math.abs(diffY) < Constants.DriveTrain.POSITION_TOLERANCE);
    }

    public void updateOdometry() {
        _odometry.update(
                getHeading(),
                _northWest.getState(),
                _southWest.getState(),
                _southEast.getState(),
                _northEast.getState());
    }

    public Pose2d getOdometryPose() {
        return _odometry.getPoseMeters();
    }

    /**
     * Reset position and gyroOffset of odometry
     *
     * @param position is a Pose2d (Translation2d, Rotation2d)
     *     <p>Translation2d resets odometry (X,Y) coordinates
     *     <p>Rotation2d - gyroAngle = gyroOffset
     *     <p>If Rotation2d <> gyroAngle, then robot heading will no longer equal IMU heading.
     */
    public void resetOdometry(Pose2d position) {
        Translation2d _translation = position.getTranslation();
        Rotation2d _rotation = getHeading();
        Pose2d _reset = new Pose2d(_translation, _rotation);
        _odometry.resetPosition(_reset, getHeading());
    }

    public void setFieldRelative(boolean relative) {
        _fieldRelative = relative;
    }

    public boolean isFieldRelative() {
        return _fieldRelative;
    }

    @Override
    public void updateDashboard() {

        metric("Swerve State", _controlState.name());
        metric("Heading State", getCurrentHeadingState().name());
        metric("Odometry Pose", getOdometryPose().toString());
    }

    public enum ControlState {
        NEUTRAL(0),
        MANUAL(1),
        POSITION(2),
        ROTATION(3),
        TRAJECTORY(4);
        private final int _value;

        ControlState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
