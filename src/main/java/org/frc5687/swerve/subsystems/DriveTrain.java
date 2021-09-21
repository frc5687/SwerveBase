/* Team 5687 (C)2020-2021 */
package org.frc5687.swerve.subsystems;

import static org.frc5687.swerve.Constants.DriveTrain.*;
import static org.frc5687.swerve.RobotMap.CAN.TALONFX.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import org.frc5687.swerve.OI;
import org.frc5687.swerve.RobotMap;
import org.frc5687.swerve.util.OutliersContainer;

public class DriveTrain extends OutliersSubsystem {
    private DiffSwerveModule _frontRight;
    private DiffSwerveModule _frontLeft;
    private DiffSwerveModule _backRight;
    private DiffSwerveModule _backLeft;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odomerty;

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

            _frontRight =
                    new DiffSwerveModule(
                            FRONT_RIGHT_POSITION,
                            FR_LEFT_FALCON,
                            FR_RIGHT_FALCON,
                            RobotMap.DIO.ENCODER_FR,
                            FRONT_RIGHT_ENCODER_OFFSET);
            _frontLeft =
                    new DiffSwerveModule(
                            FRONT_LEFT_POSITION,
                            FL_LEFT_FALCON,
                            FL_RIGHT_FALCON,
                            RobotMap.DIO.ENCODER_FL,
                            FRONT_LEFT_ENCODER_OFFSET);
            _backRight =
                    new DiffSwerveModule(
                            BACK_RIGHT_POSITION,
                            BR_LEFT_FALCON,
                            BR_RIGHT_FALCON,
                            RobotMap.DIO.ENCODER_BR,
                            BACK_RIGHT_ENCODER_OFFSET);
            _backLeft =
                    new DiffSwerveModule(
                            BACK_LEFT_POSITION,
                            BL_RIGHT_FALCON,
                            BL_LEFT_FALCON,
                            RobotMap.DIO.ENCODER_BL,
                            BACK_LEFT_ENCODER_OFFSET);

            _kinematics =
                    new SwerveDriveKinematics(
                            _frontLeft.getModulePosition(),
                            _frontRight.getModulePosition(),
                            _backLeft.getModulePosition(),
                            _backRight.getModulePosition());
            _odomerty = new SwerveDriveOdometry(_kinematics, getHeading());

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
        _frontRight.periodic();
        _frontLeft.periodic();
        _backRight.periodic();
        _backLeft.periodic();
    }

    @Override
    public void periodic() {
        _odomerty.update(
                getHeading(),
                _frontLeft.getState(),
                _frontRight.getState(),
                _backLeft.getState(),
                _backRight.getState());
    }

    @Override
    public void updateDashboard() {
        metric("BR/Encoder Angle", _backRight.getModuleAngle());
        metric("BL/Encoder Angle", _backLeft.getModuleAngle());
        metric("FL/Encoder Angle", _frontLeft.getModuleAngle());
        metric("FR/Encoder Angle", _frontRight.getModuleAngle());


        metric("BR/Predicted Angle", _backRight.getPredictedAzimuthAngle());

        metric("BR/Encoder Azimuth Vel", _backRight.getAzimuthAngularVelocity());
        metric("BR/Predicted Azimuth Vel", _backRight.getPredictedAzimuthAngularVelocity());

        metric("BR/Encoder Wheel Vel", _backRight.getWheelVelocity());
        metric("BR/Predicted Wheel Vel", _backRight.getPredictedWheelVelocity());

        metric("Odometry Pose", getOdometryPose().toString());
    }

    public void setFrontRightModuleState(SwerveModuleState state) {
        _frontRight.setIdealState(state);
    }

    public void setFrontLeftModuleState(SwerveModuleState state) {
        _frontLeft.setIdealState(state);
    }

    public void setBackLeftModuleState(SwerveModuleState state) {
        _backLeft.setIdealState(state);
    }

    public void setBackRightModuleState(SwerveModuleState state) {
        _backRight.setIdealState(state);
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
            setFrontRightModuleState(
                    new SwerveModuleState(0, new Rotation2d(_frontRight.getModuleAngle())));
            setFrontLeftModuleState(
                    new SwerveModuleState(0, new Rotation2d(_frontLeft.getModuleAngle())));
            setBackRightModuleState(
                    new SwerveModuleState(0, new Rotation2d(_backRight.getModuleAngle())));
            setBackLeftModuleState(
                    new SwerveModuleState(0, new Rotation2d(_backLeft.getModuleAngle())));
            _PIDAngle = getHeading().getRadians();
            _angleController.reset(_PIDAngle);
        } else if (Math.abs(omega) > 0) {
            SwerveModuleState[] swerveModuleStates =
                    _kinematics.toSwerveModuleStates(
                            fieldRelative
                                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                            vx, vy, omega, getHeading())
                                    : new ChassisSpeeds(vx, vy, omega));
            SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, MAX_MPS);
            setFrontLeftModuleState(swerveModuleStates[0]);
            setFrontRightModuleState(swerveModuleStates[1]);
            setBackLeftModuleState(swerveModuleStates[2]);
            setBackRightModuleState(swerveModuleStates[3]);
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
            SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, MAX_MPS);
            setFrontLeftModuleState(swerveModuleStates[0]);
            setFrontRightModuleState(swerveModuleStates[1]);
            setBackLeftModuleState(swerveModuleStates[2]);
            setBackRightModuleState(swerveModuleStates[3]);
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
                _controller.calculate(_odomerty.getPoseMeters(), goal, heading);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, MAX_MPS);
        setFrontLeftModuleState(moduleStates[0]);
        setFrontRightModuleState(moduleStates[1]);
        setBackLeftModuleState(moduleStates[2]);
        setBackRightModuleState(moduleStates[3]);
    }

    public Pose2d getOdometryPose() {
        return _odomerty.getPoseMeters();
    }

    public void startModules() {
        _frontRight.start();
        _frontLeft.start();
        _backLeft.start();
        _backRight.start();
    }
}
