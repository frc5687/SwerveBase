/* Team 5687 (C)2022 */
package org.frc5687.swerve.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5687.swerve.Constants;

// use 1323's Swerve heading controller
public class SwerveHeadingController {

    private HeadingState _headingState;
    private Rotation2d _targetHeading;
    private final PIDController _snapPID;
    private final PIDController _stabilizationPID;
    private final PIDController _visionPID;

    public SwerveHeadingController(double kDt) {
        _stabilizationPID =
                new PIDController(
                        Constants.DriveTrain.STABILIZATION_kP,
                        Constants.DriveTrain.STABILIZATION_kI,
                        Constants.DriveTrain.STABILIZATION_kD,
                        kDt);
        _snapPID =
                new PIDController(
                        Constants.DriveTrain.SNAP_kP,
                        Constants.DriveTrain.SNAP_kI,
                        Constants.DriveTrain.SNAP_kD,
                        kDt);
        _visionPID =
                new PIDController(
                        Constants.DriveTrain.VISION_kP,
                        Constants.DriveTrain.VISION_kI,
                        Constants.DriveTrain.VISION_kD,
                        kDt);
        _stabilizationPID.enableContinuousInput(-Math.PI, Math.PI);
        _snapPID.enableContinuousInput(-Math.PI, Math.PI);
        _visionPID.enableContinuousInput(-Math.PI, Math.PI);
        _targetHeading = new Rotation2d();
    }

    public HeadingState getHeadingState() {
        return _headingState;
    }

    public void setHeadingState(HeadingState state) {
        _headingState = state;
    }

    public Rotation2d getTargetHeading() {
        return _targetHeading;
    }

    public void setTargetHeading(Rotation2d targetHeading) {
        _targetHeading = targetHeading;
    }

    public double getRotationCorrection(Rotation2d heading) {
        double correction = 0;
        switch (_headingState) {
            case OFF:
                break;
            case STABILIZE:
                correction =
                        _stabilizationPID.calculate(
                                heading.getRadians(), _targetHeading.getRadians());
                break;
            case SNAP:
                correction = _snapPID.calculate(heading.getRadians(), _targetHeading.getRadians());
                break;
            case VISION:
                correction =
                        _visionPID.calculate(heading.getRadians(), _targetHeading.getRadians());
                break;
        }

        return correction;
    }

    public enum HeadingState {
        OFF(0),
        STABILIZE(1),
        SNAP(2),
        VISION(3);

        private final int _value;

        HeadingState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
