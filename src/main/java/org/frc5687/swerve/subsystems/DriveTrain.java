/* Team 5687 (C)2020-2022 */
package org.frc5687.swerve.subsystems;

import static org.frc5687.swerve.Constants.DifferentialSwerveModule.*;
import static org.frc5687.swerve.Constants.DriveTrain.*;
import static org.frc5687.swerve.util.GeometryUtil.*;
import static org.frc5687.swerve.util.SwerveHeadingController.HeadingState.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import java.util.Arrays;
import java.util.List;
import org.frc5687.swerve.Constants;
import org.frc5687.swerve.RobotMap;
import org.frc5687.swerve.util.OutliersContainer;
import org.frc5687.swerve.util.SwerveHeadingController;
import org.frc5687.swerve.util.Vector2d;

public class DriveTrain extends OutliersSubsystem {
    private DiffSwerveModule _northWest, _southWest, _northEast, _southEast;
    private List<DiffSwerveModule> _modules;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odometry;

    private Vector2d _translationVector;
    private Rotation2d _snapHeading;

    private AHRS _imu;
    private HolonomicDriveController _controller;
    private SwerveHeadingController _headingController;
    private boolean _lockHeading = false;

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

            _controller =
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
                                    Constants.DriveTrain.kP,
                                    Constants.DriveTrain.kI,
                                    Constants.DriveTrain.kD,
                                    new TrapezoidProfile.Constraints(
                                            Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
                                            Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)));
            _headingController = new SwerveHeadingController(Constants.DriveTrain.kDt);
            _translationVector = new Vector2d();
        } catch (Exception e) {
            error(e.getMessage());
        }
    }

    // use for modules as controller is running at 200Hz.
    public void controllerPeriodic() {
        _modules.forEach(DiffSwerveModule::periodic);
    }

    @Override
    public void periodic() {
        _odometry.update(
                getHeading(),
                _northWest.getState(),
                _southWest.getState(),
                _southEast.getState(),
                _northEast.getState());
    }

    @Override
    public void updateDashboard() {
        metric("SW/Velocity", _southWest.getWheelVelocity());
        metric("SW/Angle", _southWest.getModuleAngle());
        metric("SE/Velocity", _southEast.getWheelVelocity());
        metric("SE/Angle", _southEast.getModuleAngle());
        metric("NW/Velocity", _northWest.getWheelVelocity());
        metric("NW/Angle", _northWest.getModuleAngle());
        metric("NE/Velocity", _northEast.getWheelVelocity());
        metric("NE/Angle", _northEast.getModuleAngle());
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int module = 0; module < _modules.size(); module++) {
            _modules.get(module).setIdealState(states[module]);
        }
    }

    /**
     * Method to set correct module speeds and angle based on wanted vx, vy, omega
     *
     * @param vx velocity in x direction
     * @param vy velocity in y direction
     * @param omega angular velocity (rotating speed)
     * @param fieldRelative forward is always forward no mater orientation of robot.
     */
    public void drive(double vx, double vy, double omega, boolean fieldRelative) {

        metric("vx", vx);
        metric("vy", vy);
        metric("omega", omega);
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
        double scaledMagnitude = Math.pow(magnitude, POWER);
        translation =
                new Vector2d(
                        direction.getCos() * scaledMagnitude, direction.getSin() * scaledMagnitude);

        if (translation.magnitude() > 1.0) {
            translation = translation.normalize();
        }

        omega = (Math.abs(omega) < ROTATION_DEADBAND) ? 0 : omega;
        // scale rotation
        omega = Math.pow(Math.abs(omega), POWER) * Math.signum(omega);

        translation = translation.scale(MAX_MPS);
        omega *= MAX_ANG_VEL;

        _translationVector = translation;

        if (omega == 0 && _translationVector.magnitude() != 0) {
            if (!_lockHeading) {
                _headingController.setTargetHeading(getHeading());
            }
            _lockHeading = true;
        } else {
            _lockHeading = false;
            _headingController.setState(OFF);
        }

        double correctedOmega = omega + _headingController.getRotationCorrection(getHeading());
        metric("Drive X", _translationVector.x());
        metric("Drive Y", _translationVector.y());
        metric("Corrected Omega", correctedOmega);
        metric("Heading state", getCurrentHeadingState().name());
        metric("Target Heading", _headingController.getTargetHeading().getRadians());
        SwerveModuleState[] swerveModuleStates =
                _kinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        _translationVector.x(),
                                        _translationVector.y(),
                                        correctedOmega,
                                        getHeading())
                                : new ChassisSpeeds(
                                        _translationVector.x(),
                                        _translationVector.y(),
                                        correctedOmega));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED_MPS);
        //        metric("module vel", swerveModuleStates[0].speedMetersPerSecond);
        //        metric("module angle", swerveModuleStates[0].angle.getDegrees());
        setModuleStates(swerveModuleStates);
    }

    public void stabilize(Rotation2d heading) {
        _headingController.setStabilizationHeading(heading);
    }

    public void vision(Rotation2d visionHeading) {
        _headingController.setVisionHeading(visionHeading);
    }

    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, MAX_MPS);
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(MAX_MPS, MAX_MPSS)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    public void trajectoryFollower(Trajectory.State goal, Rotation2d heading) {
        ChassisSpeeds adjustedSpeeds =
                _controller.calculate(_odometry.getPoseMeters(), goal, heading);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(moduleStates);
    }

    public double getYaw() {
        return _imu.getYaw();
    }

    public SwerveHeadingController.HeadingState getCurrentHeadingState() {
        return _headingController.getHeadingState();
    }

    // yaw is negative to follow wpi coordinate system.
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-getYaw());
    }

    public Rotation2d getVisionHeading() {
        // TODO: needs to be implemented for specific robot
        return new Rotation2d();
    }

    public Rotation2d getSnapHeading() {
        return _snapHeading;
    }

    /**
     * set the heading for the robot to lock to.
     *
     * @param heading that you want to snap to.
     */
    public void setSnapHeading(Rotation2d heading) {
        _snapHeading = heading;
    }

    public void resetYaw() {
        _imu.reset();
    }

    public Pose2d getOdometryPose() {
        return _odometry.getPoseMeters();
    }

    public void startModules() {
        _modules.forEach(DiffSwerveModule::start);
    }
}
