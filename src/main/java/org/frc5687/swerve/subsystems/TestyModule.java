package org.frc5687.swerve.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frc5687.swerve.subsystems.OutliersSubsystem;
import org.frc5687.swerve.util.Helpers;
import org.frc5687.swerve.util.OutliersContainer;

public class TestyModule extends OutliersSubsystem {
    private SwerveModule _module;
    
    public TestyModule(OutliersContainer container, SwerveModule module){
        super(container);
        _module = module;
    }

    public void testModule(double x, double y){
        _module.setIdealState(new SwerveModuleState(1, new Rotation2d(x, y)));
    }

    public void shiftUp(){
        _module.shiftUp();
        // System.out.println("SHIFTING UP!!!");
    }

    public void shiftDown(){
        _module.shiftDown();
    }

    public void periodic() {
        // _module.getPosition(true);
    }

    public void updateDashboard(){
        _module.updateDashboard();
    }
}
