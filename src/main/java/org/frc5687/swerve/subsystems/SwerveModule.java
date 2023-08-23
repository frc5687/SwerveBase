/* Team 5687  */
package org.frc5687.swerve.subsystems;
import static org.frc5687.swerve.Constants.SwerveModule.*;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.swerve.Constants;
import org.frc5687.swerve.subsystems.DriveTrain.SystemIO;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusSignalValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.util.Units;


public class SwerveModule {

    private final OutliersTalon _driveMotor;
    private final OutliersTalon _turningMotor;
    private final CANCoder _encoder;
    private final double _offset;
    private final ProfiledPIDController _pidController;

    private double _absEncoderOffset;
    private double _encoderZero;
    private boolean _inverted;
    private SystemIO _SystemIO;
    private SwerveModuleState _goal;

private final StatusSignalValue<Double> _driveVelocityRotationsPerSec;
    private final StatusSignalValue<Double> _drivePositionRotations;
    private final StatusSignalValue<Double> _turningVelocityRotationsPerSec;
    private final StatusSignalValue<Double> _turningPositionRotations;

    private final BaseStatusSignalValue[] _signals;
    private double _prevAngle;
    //private ControlState _controlState;

    private boolean _hasZeroed;

    
    public SwerveModule(
        SwerveModule.ModuleConfiguration config,
        int steeringMotorID,
        int driveMotorID,
        int encoderPort
        ){
            _pidController = new ProfiledPIDController(
                Constants.SwerveModule.kP,
                Constants.SwerveModule.kI,
                Constants.SwerveModule.kD,
                 null);


            _driveMotor = new OutliersTalon(driveMotorID, config.canBus, "Drive");
            _turningMotor = new OutliersTalon(steeringMotorID, config.canBus, "Steer");

            _driveMotor.configure(Constants.SwerveModule.CONFIG);
            _turningMotor.configure(Constants.SwerveModule.CONFIG);
            _driveMotor.setTorqueCurrentFOCRate(1000);
            _turningMotor.setTorqueCurrentFOCRate(1000);

            _SystemIO = new SystemIO();
            _goal = new SwerveModuleState(0.0, getCanCoderAngle());


            _encoder = new CANCoder(encoderPort, config.canBus);
            CANCoderConfiguration CANfig = new CANCoderConfiguration();
            // set units of the CANCoder to radians, with velocity being radians per second
            CANfig.sensorCoefficient = 2 * Math.PI / 4096.0;
            CANfig.unitString = "rad";
            CANfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
            CANfig.sensorTimeBase = SensorTimeBase.PerSecond;
            CANfig.magnetOffsetDegrees = Constants.SwerveModule.CAN_OFFSET;
            _encoder.configAllSettings(CANfig);

            _offset = Constants.SwerveModule.CAN_OFFSET;
            
            _drivePositionRotations = _driveMotor.getPosition();
            _driveVelocityRotationsPerSec = _driveMotor.getVelocity();
            _turningPositionRotations = _turningMotor.getPosition();
            _turningVelocityRotationsPerSec = _turningMotor.getVelocity();

            _driveVelocityRotationsPerSec.setUpdateFrequency(1 / kDt);
            _drivePositionRotations.setUpdateFrequency(1 / kDt);

           _turningVelocityRotationsPerSec.setUpdateFrequency(1 / kDt);
           _turningPositionRotations.setUpdateFrequency(1 / kDt);

           _signals = new BaseStatusSignalValue[4];
           _signals[0] = _driveVelocityRotationsPerSec;
           _signals[1] = _drivePositionRotations;
           _signals[2] = _turningVelocityRotationsPerSec;
           _signals[3] = _turningPositionRotations;
           //_controlState = ControlState.OFF;
           _hasZeroed = false;   
        }
        // public ControlState getControlState() {
        //     return _controlState;
        // }
        // public void setControlState(ControlState state) {
        //     _controlState = state;
        // }
        public BaseStatusSignalValue[] getSignals() {
            return _signals;
        }
        public void setIdealState( SwerveModuleState state){
            if (state.speedMetersPerSecond < 0.1 ){
            StopAll();
            } else {
            state = SwerveModuleState.optimize(state, getCanCoderAngle());
            _goal = state;
            setModuleState(_goal);
            }
        }
        public void setModuleState(SwerveModuleState state){
            _driveMotor.setPercentOutput(state.speedMetersPerSecond / Constants.SwerveModule.MAX_SPEED);
            _turningMotor.setPercentOutput(_pidController.calculate(getCanCoderAngle().getDegrees(), state.angle.getDegrees()));

        }
        public SwerveModuleState getState(){
            return new SwerveModuleState(getWheelVelocity(), getCanCoderAngle());
        }
        public void StopAll(){
            _driveMotor.stopMotor();
            _turningMotor.stopMotor();
        }
        public double getEncoderAngledouble(){
            return _encoder.getAbsolutePosition();
        }
        
        public Rotation2d getCanCoderAngle() {
            return Rotation2d.fromDegrees(_encoder.getAbsolutePosition());
        }
        public double getDriveRPM(){
            return OutliersTalon.rotationsPerSecToRPM(_driveVelocityRotationsPerSec.getValue(), 1.0);
        }
        public double getTurningRPM(){
            return OutliersTalon.rotationsPerSecToRPM(_turningVelocityRotationsPerSec.getValue(), 1.0);
        }
        public double getWheelVelocity() {
            return getWheelAngularVelocity() * Constants.SwerveModule.WHEEL_RADIUS; // Meters per sec.
        }
        public double getWheelAngularVelocity(){
            return Units.rotationsPerMinuteToRadiansPerSecond(getDriveRPM() / Constants.SwerveModule.GEAR_RATIO_DRIVE);
        }
        

    public static class ModuleConfiguration {
        public String moduleName = "";

        public Translation2d position = new Translation2d();

        public double encoderOffset = 0.0;
        public boolean encoderInverted = false;

        public String canBus = "rio";
    }
    // public enum ControlState {
    //     OFF(0),
    //     STATE_CONTROL(1),
    //     CHARACTERIZATION(2);
    //     private final int _value;

    //     ControlState(int value) {
    //         _value = value;
    //     }

    //     public int getValue() {
    //         return _value;
    //     }
    // }


}
