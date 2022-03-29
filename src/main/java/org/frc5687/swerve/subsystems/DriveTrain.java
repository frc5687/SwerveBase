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
import edu.wpi.first.math.geometry.Translation2d;
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

public class DriveTrain extends OutliersSubsystem {
    private DiffSwerveModule _northWest, _southWest, _northEast, _southEast;
    private List<DiffSwerveModule> _modules;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odometry;

    private Translation2d _translationVector;
    private Rotation2d _snapHeading;

    private AHRS _imu;
    private HolonomicDriveController _controller;
    private SwerveHeadingController _headingController;

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
                            Constants.DriveTrain.NORTH_WEST_ENCODER_INVERTED);
            _southWest =
                    new DiffSwerveModule(
                            Constants.DriveTrain.SOUTH_WEST,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_OUTER,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_INNER,
                            RobotMap.DIO.SOUTH_WEST,
                            Constants.DriveTrain.SOUTH_WEST_OFFSET,
                            Constants.DriveTrain.SOUTH_WEST_ENCODER_INVERTED);
            _southEast =
                    new DiffSwerveModule(
                            Constants.DriveTrain.SOUTH_EAST,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_INNER,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_OUTER,
                            RobotMap.DIO.SOUTH_EAST,
                            Constants.DriveTrain.SOUTH_EAST_OFFSET,
                            Constants.DriveTrain.SOUTH_EAST_ENCODER_INVERTED);
            _northEast =
                    new DiffSwerveModule(
                            Constants.DriveTrain.NORTH_EAST,
                            RobotMap.CAN.TALONFX.NORTH_EAST_INNER,
                            RobotMap.CAN.TALONFX.NORTH_EAST_OUTER,
                            RobotMap.DIO.NORTH_EAST,
                            Constants.DriveTrain.NORTH_EAST_OFFSET,
                            Constants.DriveTrain.NORTH_EAST_ENCODER_INVERTED);

            _modules = Arrays.asList(_northWest, _southWest, _southEast, _northWest);

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
            _translationVector = new Translation2d();
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
    public void updateDashboard() {}

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
    public void drive(
            double vx,
            double vy,
            double omega,
            boolean fieldRelative,
            boolean useVision,
            boolean useSnap) {

        metric("vx", vx);
        metric("vy", vy);
        metric("omega", omega);
        Translation2d translation = new Translation2d(vx, vy);
        Rotation2d direction = getDirection(translation);

        double magnitude = translation.getNorm();

        if (Math.abs(getDistance(direction, getNearestPole(direction))) < POLE_THRESHOLD) {
            translation = rotationToTranslation(getNearestPole(direction)).times(magnitude);
        }

        if (magnitude < TRANSLATION_DEADBAND) {
            translation = new Translation2d();
            magnitude = 0;
        }

        direction = getDirection(translation);
        double scaledMagnitude = Math.pow(magnitude, POWER);
        translation =
                new Translation2d(
                        direction.getCos() * scaledMagnitude, direction.getSin() * scaledMagnitude);

        omega = (Math.abs(omega) < ROTATION_DEADBAND) ? 0 : omega;
        // scale rotation
        omega = Math.pow(Math.abs(omega), POWER) * Math.signum(omega);

        translation = translation.times(MAX_MPS);
        omega *= MAX_ANG_VEL;

        if (omega == 0 && _translationVector.getNorm() != 0) {
            _headingController.setTargetHeading(getHeading());
            _headingController.setHeadingState(STABILIZE);
        } else if (useSnap) {
            _headingController.setTargetHeading(getSnapHeading());
            _headingController.setHeadingState(SNAP);
        } else if (useVision) {
            _headingController.setTargetHeading(getVisionHeading());
            _headingController.setHeadingState(VISION);
        } else {
            _headingController.setHeadingState(OFF);
        }

        _translationVector = translation;

        double correctedOmega = omega + _headingController.getRotationCorrection(getHeading());
        metric("Drive X", _translationVector.getX());
        metric("Drive Y", _translationVector.getY());
        metric("Corrected Omega", correctedOmega);
        SwerveModuleState[] swerveModuleStates =
                _kinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        _translationVector.getX(),
                                        _translationVector.getY(),
                                        correctedOmega,
                                        getHeading())
                                : new ChassisSpeeds(
                                        _translationVector.getX(),
                                        _translationVector.getY(),
                                        correctedOmega));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(swerveModuleStates);
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
