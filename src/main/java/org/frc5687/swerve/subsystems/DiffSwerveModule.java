/* Team 5687 (C)5687-2022 */
package org.frc5687.swerve.subsystems;

import static org.frc5687.swerve.Constants.DifferentialSwerveModule.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.frc5687.swerve.util.Helpers;

/**
 * Created 10/11/2020 by Dennis Slobodzian.
 *
 * <p>Wrapper class for a differential swerve module using LQR as the controller for azimuth angle
 * and wheel velocity of the module.
 */
public class DiffSwerveModule {
    private final TalonFX _rightFalcon;
    private final TalonFX _leftFalcon;
    private final DutyCycleEncoder _boreEncoder;
    private final Translation2d _positionVector;
    private final LinearSystemLoop<N3, N2, N3> _swerveControlLoop;
    private Matrix<N3, N1> _reference; // same thing as a set point.
    private Matrix<N2, N1> _u;

    private final double _encoderOffset;
    private final boolean _encoderInverted;
    private boolean _running;

    private final SystemIO _systemIO;

    public DiffSwerveModule(
            Translation2d positionVector,
            int leftMotorID,
            int rightMotorID,
            int encoderNum,
            double encoderOffset,
            boolean encoderInverted,
            String canBus) {
        // setup azimuth bore encoder.
        _boreEncoder = new DutyCycleEncoder(encoderNum);
        _boreEncoder.setDistancePerRotation(2.0 * Math.PI);
        _encoderOffset = encoderOffset;
        _encoderInverted = encoderInverted;

        _reference = Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 0);
        _positionVector = positionVector;
        // setup both falcon motors.
        _leftFalcon = new TalonFX(leftMotorID, canBus);
        _rightFalcon = new TalonFX(rightMotorID, canBus);

        configureMotor(_leftFalcon);
        configureMotor(_rightFalcon);

        // Creates a Linear System of our Differential Swerve Module.
        LinearSystem<N3, N2, N3> swerveModuleModel =
                createDifferentialSwerveModule(
                        DCMotor.getFalcon500(2),
                        INERTIA_STEER,
                        INERTIA_WHEEL,
                        GEAR_RATIO_STEER,
                        GEAR_RATIO_WHEEL);

        // Creates a Kalman Filter as our Observer for our module. Works since system is linear.
        KalmanFilter<N3, N2, N3> swerveObserver =
                new KalmanFilter<>(
                        Nat.N3(),
                        Nat.N3(),
                        swerveModuleModel,
                        Matrix.mat(Nat.N3(), Nat.N1())
                                .fill(
                                        MODEL_AZIMUTH_ANGLE_NOISE,
                                        MODEL_AZIMUTH_ANG_VELOCITY_NOISE,
                                        MODEL_WHEEL_ANG_VELOCITY_NOISE),
                        Matrix.mat(Nat.N3(), Nat.N1())
                                .fill(
                                        SENSOR_AZIMUTH_ANGLE_NOISE,
                                        SENSOR_AZIMUTH_ANG_VELOCITY_NOISE,
                                        SENSOR_WHEEL_ANG_VELOCITY_NOISE),
                        kDt);
        // Creates an LQR controller for our Swerve Module.
        LinearQuadraticRegulator<N3, N2, N3> swerveController =
                new LinearQuadraticRegulator<>(
                        swerveModuleModel,
                        // Q Vector/Matrix Maximum error tolerance
                        VecBuilder.fill(Q_AZIMUTH, Q_AZIMUTH_ANG_VELOCITY, Q_WHEEL_ANG_VELOCITY),
                        // R Vector/Matrix Maximum control effort.
                        VecBuilder.fill(CONTROL_EFFORT, CONTROL_EFFORT),
                        kDt);

        // Creates a LinearSystemLoop that contains the Model, Controller, Observer, Max Volts,
        // Update Rate.
        _swerveControlLoop =
                new LinearSystemLoop<>(
                        swerveModuleModel, swerveController, swerveObserver, VOLTAGE, kDt);
        //         Initializes the vectors and matrices.
        _swerveControlLoop.reset(VecBuilder.fill(0, 0, 0));
        _u = VecBuilder.fill(0, 0);
        // boolean for if we want the modules to be running as we set voltage in the periodic loop.
        _running = false;
        _systemIO = new SystemIO();
    }

    /**
     * configure falcon500 motor
     *
     * @param falcon motor
     */
    private void configureMotor(TalonFX falcon) {
        falcon.configFactoryDefault();
        falcon.setInverted(false);
        falcon.setSensorPhase(false);
        // Model assumes the motors are in brake mode so do not change.
        falcon.setNeutralMode(NeutralMode.Brake);
        falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TIMEOUT);
        falcon.configForwardSoftLimitEnable(false);
        falcon.configVoltageCompSaturation(VOLTAGE, TIMEOUT);
        falcon.enableVoltageCompensation(true);
        falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, TIMEOUT);
        falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, TIMEOUT);
        falcon.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms, TIMEOUT);
        falcon.configVelocityMeasurementWindow(FALCON_VELOCITY_MEASUREMENT_WINDOW, TIMEOUT);
        falcon.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        ENABLE_CURRENT_LIMIT,
                        CURRENT_LIMIT,
                        CURRENT_THRESHOLD,
                        CURRENT_TRIGGER_TIME));
    }
    /**
     * wraps angle so that absolute encoder can be continuous. (i.e) No issues when switching
     * between -PI and PI as they are the same point but different values.
     *
     * @param reference is the Matrix that contains the reference wanted such as [Math.PI, 0, 100].
     * @param xHat is the predicted states of our system. [Azimuth Angle, Azimuth Angular Velocity,
     *     Wheel Angular Velocity].
     */
    private Matrix<N3, N1> wrapAngle(Matrix<N3, N1> reference, Matrix<N3, N1> xHat) {
        double angleError = reference.get(0, 0) - getModuleAngle();
        double positionError = MathUtil.inputModulus(angleError, -Math.PI, Math.PI);
        Matrix<N3, N1> error = reference.minus(xHat);
        return VecBuilder.fill(positionError, error.get(1, 0), error.get(2, 0));
    }

    public synchronized void readInputs() {
        _systemIO.leftVelocityTicksPer100ms = _leftFalcon.getSelectedSensorVelocity(0);
        _systemIO.rightVelocityTicksPer100ms = _rightFalcon.getSelectedSensorVelocity(0);

        _systemIO.moduleAngle = getEncoderAngle();
        _systemIO.moduleAzimuthAngularVelocity = getAzimuthAngularVelocity();
        _systemIO.moduleWheelAngularVelocity = getWheelAngularVelocity();
    }
    // periodic loop runs at 5ms.
    public void periodic() {
        // read inputs from falcon;
        readInputs();
        // sets the next reference / setpoint.
        _swerveControlLoop.setNextR(_reference);
        // updates the kalman filter with new data points.
        _swerveControlLoop.correct(
                VecBuilder.fill(
                        getModuleAngle(), getAzimuthAngularVelocity(), getWheelAngularVelocity()));
        // predict step of kalman filter.
        predict();
        if (_running) {
            setLeftFalconVoltage(getLeftNextVoltage());
            setRightFalconVoltage(getRightNextVoltage());
        }
    }

    // use custom predict() function for as absolute encoder azimuth angle and the angular velocity
    // of the module need to be continuous.
    private void predict() {
        // creates our input of voltage to our motors of u = K(r-x) but need to wrap angle to be
        // continuous
        _u =
                _swerveControlLoop.clampInput(
                        _swerveControlLoop
                                .getController()
                                .getK()
                                .times( // profiledReference())
                                        wrapAngle(
                                                _swerveControlLoop.getNextR(),
                                                _swerveControlLoop.getXHat()))
                                .plus(
                                        _swerveControlLoop
                                                .getFeedforward()
                                                .calculate(_swerveControlLoop.getNextR())));
        _swerveControlLoop.getObserver().predict(_u, kDt);
    }

    public void start() {
        _running = true;
    }

    public void stop() {
        _running = false;
    }

    public Translation2d getModulePosition() {
        return _positionVector;
    }

    public void setRightFalconVoltage(double voltage) {
        double limVoltage = Helpers.limit(voltage, -VOLTAGE, VOLTAGE);
        _rightFalcon.set(TalonFXControlMode.PercentOutput, limVoltage / VOLTAGE);
    }

    public void setLeftFalconVoltage(double voltage) {
        double limVoltage = Helpers.limit(voltage, -VOLTAGE, VOLTAGE);
        _leftFalcon.set(TalonFXControlMode.PercentOutput, limVoltage / VOLTAGE);
    }

    public double getEncoderAngle() {
        return Helpers.boundHalfAngle(
                ((_encoderInverted ? (-1.0) : 1.0) * _boreEncoder.getDistance() % (2.0 * Math.PI))
                        - _encoderOffset,
                true);
    }

    public double getModuleAngle() {
        return _systemIO.moduleAngle;
    }

    public double getWheelAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                        getLeftFalconRPM() / GEAR_RATIO_WHEEL
                                - getRightFalconRPM() / GEAR_RATIO_WHEEL)
                / 2.0;
    }

    public double getWheelVelocity() {
        return getWheelAngularVelocity() * WHEEL_RADIUS; // Meters per sec.
    }

    public double getPredictedWheelVelocity() {
        return getPredictedWheelAngularVelocity() * WHEEL_RADIUS;
    }

    public double getAzimuthAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                        getLeftFalconRPM() / GEAR_RATIO_STEER
                                + getRightFalconRPM() / GEAR_RATIO_STEER)
                / 2.0;
    }

    public double getRightFalconRPM() {
        return _systemIO.rightVelocityTicksPer100ms / TICKS_TO_ROTATIONS * FALCON_RATE;
    }

    public double getLeftFalconRPM() {
        return _systemIO.leftVelocityTicksPer100ms / TICKS_TO_ROTATIONS * FALCON_RATE;
    }

    public double getLeftVoltage() {
        return _leftFalcon.getMotorOutputVoltage();
    }

    public double getRightVoltage() {
        return _rightFalcon.getMotorOutputVoltage();
    }

    public double getPredictedAzimuthAngularVelocity() {
        return _swerveControlLoop.getObserver().getXhat(1);
    }

    public double getPredictedWheelAngularVelocity() {
        return _swerveControlLoop.getXHat(2);
    }

    public double getPredictedAzimuthAngle() {
        return _swerveControlLoop.getXHat(0);
    }

    public double getReferenceWheelAngularVelocity() {
        return _swerveControlLoop.getNextR(2);
    }

    public void setReference(Matrix<N3, N1> reference) {
        _reference = reference;
    }

    /**
     * gets the wanted voltage from our control law. u = K(r-x) our control law is slightly
     * different as we need to be continuous. Check method predict() for calculations.
     *
     * @return left wanted voltage
     */
    public double getLeftNextVoltage() {
        return _u.get(0, 0);
    }

    public double getRightNextVoltage() {
        return _u.get(1, 0);
    }

    public double getLeftCurrent() {
        return _leftFalcon.getSupplyCurrent();
    }

    public double getRightCurrent() {
        return _rightFalcon.getSupplyCurrent();
    }

    public double getReferenceModuleAngle() {
        return _swerveControlLoop.getNextR(0);
    }

    public double getReferenceModuleAngularVelocity() {
        return _swerveControlLoop.getNextR(1);
    }

    public double getReferenceWheelVelocity() {
        return _swerveControlLoop.getNextR(2);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelVelocity(), new Rotation2d(getModuleAngle()));
    }

    /**
     * Sets the state of the module and sends the voltages wanted to the motors.
     *
     * @param state is the desired swerve module state.
     */
    public void setModuleState(SwerveModuleState state) {
        if (state.speedMetersPerSecond != 0) {
            setReference(
                    VecBuilder.fill(
                            state.angle.getRadians(),
                            0,
                            state.speedMetersPerSecond / WHEEL_RADIUS));
        } else {
            setReference(
                    VecBuilder.fill(
                            getModuleAngle(), 0, state.speedMetersPerSecond / WHEEL_RADIUS));
        }
    }

    /**
     * sets the modules to take the shorted path to the newest state.
     *
     * @param state azimuth angle in radians and velocity of wheel in meters per sec.
     */
    public void setIdealState(SwerveModuleState state) {
        setModuleState(SwerveModuleState.optimize(state, new Rotation2d(getModuleAngle())));
    }

    /**
     * Creates a StateSpace model of a differential swerve module.
     *
     * @param motor is the motor used.
     * @param Js is the Moment of Inertia of the steer component.
     * @param Jw is the Moment of Inertia of the wheel component.
     * @param Gs is the Gear Ratio of the steer.
     * @param Gw is the Gear Ratio of the wheel.
     * @return LinearSystem of state space model.
     */
    private static LinearSystem<N3, N2, N3> createDifferentialSwerveModule(
            DCMotor motor, double Js, double Jw, double Gs, double Gw) {
        var Cs = -((Gs * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Js));
        var Cw = -((Gw * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Jw));
        var Vs = 0.5 * ((Gs * motor.KtNMPerAmp) / (motor.rOhms * Js));
        var Vw = 0.5 * ((Gw * motor.KtNMPerAmp) / (motor.rOhms * Jw));

        var A =
                Matrix.mat(Nat.N3(), Nat.N3())
                        .fill(0.0, 1.0, 0.0, 0.0, Gs * Cs, 0.0, 0.0, 0.0, Gw * Cw);
        var B = Matrix.mat(Nat.N3(), Nat.N2()).fill(0.0, 0.0, Vs, Vs, Vw, -Vw);
        var C = Matrix.mat(Nat.N3(), Nat.N3()).fill(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        var D =
                Matrix.mat(Nat.N3(), Nat.N2())
                        .fill(
                                0.0, 0.0,
                                0.0, 0.0,
                                0.0, 0.0);
        return new LinearSystem<>(A, B, C, D);
    }

    private static class SystemIO {
        // Falcon 500 sensor inputs
        public double leftVelocityTicksPer100ms;
        public double rightVelocityTicksPer100ms;
        // State Space Sensor Inputs
        public double moduleAngle;
        public double moduleAzimuthAngularVelocity;
        public double moduleWheelAngularVelocity;
    }
}
