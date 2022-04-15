/* Team 5687 (C)2021-2022 */
package org.frc5687.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.swerve.commands.Drive;
import org.frc5687.swerve.commands.OutliersCommand;
import org.frc5687.swerve.subsystems.Coprocessor;
import org.frc5687.swerve.subsystems.DriveTrain;
import org.frc5687.swerve.subsystems.OutliersSubsystem;
import org.frc5687.swerve.util.OutliersContainer;
import org.frc5687.swerve.util.PeriodicManager;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private Coprocessor _coprocessor;

    private Robot _robot;
    private DriveTrain _driveTrain;

    private PeriodicManager _periodicManager;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _coprocessor = new Coprocessor();

        _driveTrain = new DriveTrain(this, _imu, _coprocessor);

        _periodicManager = new PeriodicManager(_coprocessor, _driveTrain);

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));

        _periodicManager.startPeriodic();
        _imu.reset();
    }

    public void periodic() {}

    public void disabledPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void autonomousInit() {}

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }
}
