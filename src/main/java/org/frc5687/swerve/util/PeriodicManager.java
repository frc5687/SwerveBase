/* Team 5687 (C)2022 */
package org.frc5687.swerve.util;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import org.frc5687.swerve.Constants;

public final class PeriodicManager {
    private final List<OutlierPeriodic> _periods = new ArrayList<>();

    private double _controlPrevTimestamp;
    private double _dataPrevTimestamp;

    private double _controlDt;
    private double _dataDt;

    private final Notifier _controlThread =
            new Notifier(
                    () -> {
                        synchronized (PeriodicManager.this) {
                            final double timestamp = Timer.getFPGATimestamp();
                            _controlDt = timestamp - _controlPrevTimestamp;
                            _controlPrevTimestamp = timestamp;
                            _periods.forEach(p -> p.controlPeriodic(timestamp));
                        }
                    });
    private final Notifier _dataThread =
            new Notifier(
                    () -> {
                        synchronized (PeriodicManager.this) {
                            final double timestamp = Timer.getFPGATimestamp();
                            _dataDt = timestamp - _dataPrevTimestamp;
                            _dataPrevTimestamp = timestamp;
                            _periods.forEach(p -> p.dataPeriodic(timestamp));
                        }
                    });

    public PeriodicManager(OutlierPeriodic... periods) {
        this._periods.addAll(List.of(periods));
    }

    public void startPeriodic() {
        _controlThread.startPeriodic(Constants.CONTROL_PERIOD);
        _dataThread.startPeriodic(Constants.DATA_PERIOD);
    }

    public void stopPeriodic() {
        _controlThread.stop();
        _dataThread.stop();
    }

    public void outputToDashboard() {
        SmartDashboard.putNumber("Periodic Control DT", _controlDt);
        SmartDashboard.putNumber("Periodic Data DT", _dataDt);
    }
}
