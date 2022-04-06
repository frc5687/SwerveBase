/* Team 5687 (C)2021-2022 */
package org.frc5687.swerve.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5687.swerve.OI;
import org.frc5687.swerve.subsystems.DriveTrain;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;

    private final OI _oi;
    private final SlewRateLimiter _vxLimiter;
    private final SlewRateLimiter _vyLimiter;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _vxLimiter = new SlewRateLimiter(4); // units per sec
        _vyLimiter = new SlewRateLimiter(4); // units per sec
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.startModules();
    }

    @Override
    public void execute() {
        super.execute();
        //  driveX and driveY are swapped due to coordinate system that WPILib uses.
        double x = _vxLimiter.calculate(_oi.getDriveY());
        double y = _vyLimiter.calculate(_oi.getDriveX());
        double omega = _oi.getRotationX();
        if (_oi.snap()) {
            _driveTrain.setSnapHeading(new Rotation2d(Math.PI / 2.0));
        } else if (_oi.autoAim()) {
            _driveTrain.vision(_driveTrain.getVisionHeading());
        }
        _driveTrain.drive(x, y, omega, true);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
