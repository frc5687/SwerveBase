/* Team 5687 (C)2021-2022 */
package org.frc5687.swerve.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5687.swerve.OI;
import org.frc5687.swerve.subsystems.DriveTrain;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;

    private final OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
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
        double x = _oi.getDriveY();
        double y = _oi.getDriveX();
        double omega = _oi.getRotationX();
        if (_oi.snap()) {
            _driveTrain.setSnapHeading(new Rotation2d(Math.PI / 2.0));
            _driveTrain.drive(x, y, omega, true, false, true);
        } else if (_oi.autoAim()) {
            _driveTrain.drive(x, y, omega, true, true, false);
        } else {
            _driveTrain.drive(x, y, omega, true, false, false);
        }
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
