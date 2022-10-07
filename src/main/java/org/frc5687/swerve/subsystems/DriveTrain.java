/* Team 5687 (C)2020-2022 */
package org.frc5687.swerve.subsystems;

import static org.frc5687.swerve.Constants.DifferentialSwerveModule.*;
import static org.frc5687.swerve.Constants.DriveTrain.*;
import static org.frc5687.swerve.RobotMap.CAN.TALONFX.*;

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
import org.frc5687.swerve.OI;
import org.frc5687.swerve.RobotMap;
import org.frc5687.swerve.util.OutliersContainer;

public class DriveTrain extends OutliersSubsystem {
    private DiffSwerveModule _northEast;
    private DiffSwerveModule _northWest;
    private DiffSwerveModule _southEast;
    private DiffSwerveModule _southWest;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odometry;

    private double _PIDAngle;

    private AHRS _imu;
    private OI _oi;

    private HolonomicDriveController _controller;
    private ProfiledPIDController _angleController;

    public DriveTrain(OutliersContainer container, OI oi, AHRS imu) {
        super(container);
        try {
            _oi = oi;
            _imu = imu;

            _northEast =
                    new DiffSwerveModule(
                            NORTH_EAST_POSITION,
                            NE_LEFT_FALCON,
                            NE_RIGHT_FALCON,
                            RobotMap.DIO.ENCODER_NE,
                            NORTH_EAST_ENCODER_OFFSET);
            _northWest =
                    new DiffSwerveModule(
                            NORTH_WEST_POSITION,
                            NW_LEFT_FALCON,
                            NW_RIGHT_FALCON,
                            RobotMap.DIO.ENCODER_NW,
                            NORTH_WEST_ENCODER_OFFSET);
            _southEast =
                    new DiffSwerveModule(
                            SOUTH_EAST_POSITION,
                            SE_LEFT_FALCON,
                            SE_RIGHT_FALCON,
                            RobotMap.DIO.ENCODER_SE,
                            SOUTH_EAST_ENCODER_OFFSET);
            _southWest =
                    new DiffSwerveModule(
                            SOUTH_WEST_POSITION,
                            SW_RIGHT_FALCON,
                            SW_LEFT_FALCON,
                            RobotMap.DIO.ENCODER_SW,
                            SOUTH_WEST_ENCODER_OFFSET);

            _kinematics =
                    new SwerveDriveKinematics(
                            _northWest.getModulePosition(),
                            _northEast.getModulePosition(),
                            _southWest.getModulePosition(),
                            _southEast.getModulePosition());
            _odometry = new SwerveDriveOdometry(_kinematics, getHeading());

            _controller =
                    new HolonomicDriveController(
                            new PIDController(kP, kI, kD),
                            new PIDController(kP, kI, kD),
                            new ProfiledPIDController(
                                    kP,
                                    kI,
                                    kD,
                                    new TrapezoidProfile.Constraints(
                                            PROFILE_CONSTRAINT_VEL, PROFILE_CONSTRAINT_ACCEL)));
            _angleController =
                    new ProfiledPIDController(
                            ANGLE_kP,
                            ANGLE_kI,
                            ANGLE_kD,
                            new TrapezoidProfile.Constraints(
                                    PROFILE_CONSTRAINT_VEL, PROFILE_CONSTRAINT_ACCEL));
            _angleController.enableContinuousInput(-Math.PI / 2.0, Math.PI / 2.0);
        } catch (Exception e) {
            error(e.getMessage());
        }
    }

    // use for modules as controller is running at 200Hz.
    public void controllerPeriodic() {
        _northEast.periodic();
        _northWest.periodic();
        _southEast.periodic();
        _southWest.periodic();
    }

    @Override
    public void periodic() {
        _odometry.update(
                getHeading(),
                _northWest.getState(),
                _northEast.getState(),
                _southWest.getState(),
                _southEast.getState());
    }

    @Override
    public void updateDashboard() {
        metric("SE/Encoder Angle", _southEast.getModuleAngle());
        metric("SW/Encoder Angle", _southWest.getModuleAngle());
        metric("NW/Encoder Angle", _northWest.getModuleAngle());
        metric("NE/Encoder Angle", _northEast.getModuleAngle());

        metric("SE/Predicted Angle", _southEast.getPredictedAzimuthAngle());

        metric("SE/Encoder Azimuth Vel", _southEast.getAzimuthAngularVelocity());
        metric("SE/Predicted Azimuth Vel", _southEast.getPredictedAzimuthAngularVelocity());

        metric("SE/Encoder Wheel Vel", _southEast.getWheelVelocity());
        metric("SE/Predicted Wheel Vel", _southEast.getPredictedWheelVelocity());

        metric("Odometry Pose", getOdometryPose().toString());
    }

    public void setNorthEastModuleState(SwerveModuleState state) {
        _northEast.setIdealState(state);
    }

    public void setNorthWestModuleState(SwerveModuleState state) {
        _northWest.setIdealState(state);
    }

    public void setSouthWestModuleState(SwerveModuleState state) {
        _southWest.setIdealState(state);
    }

    public void setSouthEastModuleState(SwerveModuleState state) {
        _southEast.setIdealState(state);
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
        if (Math.abs(vx) < DEADBAND && Math.abs(vy) < DEADBAND && Math.abs(omega) < DEADBAND) {
            setNorthEastModuleState(
                    new SwerveModuleState(0, new Rotation2d(_northEast.getModuleAngle())));
            setNorthWestModuleState(
                    new SwerveModuleState(0, new Rotation2d(_northWest.getModuleAngle())));
            setSouthEastModuleState(
                    new SwerveModuleState(0, new Rotation2d(_southEast.getModuleAngle())));
            setSouthWestModuleState(
                    new SwerveModuleState(0, new Rotation2d(_southWest.getModuleAngle())));
            _PIDAngle = getHeading().getRadians();
            _angleController.reset(_PIDAngle);
        } else if (Math.abs(omega) > 0) {
            SwerveModuleState[] swerveModuleStates =
                    _kinematics.toSwerveModuleStates(
                            fieldRelative
                                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                            vx, vy, omega, getHeading())
                                    : new ChassisSpeeds(vx, vy, omega));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED_MPS);
            setNorthWestModuleState(swerveModuleStates[0]);
            setNorthEastModuleState(swerveModuleStates[1]);
            setSouthWestModuleState(swerveModuleStates[2]);
            setSouthEastModuleState(swerveModuleStates[3]);
            _PIDAngle = getHeading().getRadians();
            _angleController.reset(_PIDAngle);
        } else {
            SwerveModuleState[] swerveModuleStates =
                    _kinematics.toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    vx,
                                    vy,
                                    _angleController.calculate(
                                            getHeading().getRadians(), _PIDAngle),
                                    new Rotation2d(_PIDAngle)));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED_MPS);
            setNorthWestModuleState(swerveModuleStates[0]);
            setNorthEastModuleState(swerveModuleStates[1]);
            setSouthWestModuleState(swerveModuleStates[2]);
            setSouthEastModuleState(swerveModuleStates[3]);
        }
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
        setNorthWestModuleState(moduleStates[0]);
        setNorthEastModuleState(moduleStates[1]);
        setSouthWestModuleState(moduleStates[2]);
        setSouthEastModuleState(moduleStates[3]);
    }

    public Pose2d getOdometryPose() {
        return _odometry.getPoseMeters();
    }

    public void startModules() {
        _northEast.start();
        _northWest.start();
        _southWest.start();
        _southEast.start();
    }
}
