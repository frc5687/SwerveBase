/* Team 5687 (C)2022 */
package org.frc5687.lib.drivers;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * TalonFX wrapper class that uses 254's LazyTalonFX that reduces CAN bus / CPU
 * overhead by skipping
 * duplicate set commands. (By default the Talon flushes the Tx buffer on every
 * set call).
 */
public class OutliersTalon extends TalonFX {
    // private final String _name;
    private final TalonFXConfigurator _configurator;
    private TalonFXConfiguration _configuration = new TalonFXConfiguration();

    private ClosedLoopGeneralConfigs _closedLoopGenConfig = new ClosedLoopGeneralConfigs();

    private Slot0Configs _slot0Configs = new Slot0Configs();
    private Slot1Configs _slot1Configs = new Slot1Configs();
    private MotorOutputConfigs _motorConfigs = new MotorOutputConfigs();
    private TorqueCurrentConfigs _torqueCurrentConfigs = new TorqueCurrentConfigs();
    private final VoltageConfigs _voltageConfigs = new VoltageConfigs();
    private final MotionMagicConfigs _motionMagicConfigs = new MotionMagicConfigs();
    private final CurrentLimitsConfigs _currentLimitsConfigs = new CurrentLimitsConfigs();
    private final FeedbackConfigs _feedbackConfigs = new FeedbackConfigs();

    private final DutyCycleOut _percentOutput = new DutyCycleOut(0.0);
    private final TorqueCurrentFOC _torqueCurrentFOC = new TorqueCurrentFOC(0.0);
    private final VoltageOut _voltageOut = new VoltageOut(0.0);
    private final MotionMagicVoltage _motionMagicVoltage = new MotionMagicVoltage(0.0);
    private final VelocityVoltage _velocityVoltage = new VelocityVoltage(0.0, true, 0, 0, false);

    public OutliersTalon(int port, String canBus, String name) {
        super(port, canBus);
        _configurator = this.getConfigurator();
        _configurator.apply(_configuration);
        setPercentOutput(0.0);
        // _name = name;
    }

    public void setPercentOutput(double output) {
        if (_percentOutput.Output != output) {
            this.setControl(_percentOutput.withOutput(output));
        }
    }

    public void setVoltage(double voltage) {
        if (_voltageOut.Output != voltage) {
            this.setControl(_voltageOut.withOutput(voltage));
        }
    }

    public void setMotionMagic(double position) {
        if (_motionMagicVoltage.Position != position) {
            this.setControl(_motionMagicVoltage.withPosition(position).withSlot(0));
        }
    }

    public void setVelocity(double rpm) {
        if (_velocityVoltage.Velocity != rpm) {
            this.setControl(_velocityVoltage.withVelocity(rpm));
        }
    }

    public void setTorqueCurrentFOCRate(double hz) {
        _torqueCurrentFOC.withUpdateFreqHz(hz);
    }

    public void setTorqueCurrentFOC(double current) {
        if (_torqueCurrentFOC.Output != current) {
            this.setControl(_torqueCurrentFOC.withOutput(current));
        }
    }

    public void configure(Configuration config) {
        _motorConfigs.Inverted = config.INVERTED;
        _motorConfigs.NeutralMode = config.NEUTRAL_MODE;

        _currentLimitsConfigs.StatorCurrentLimit = config.MAX_STATOR_CURRENT;
        _currentLimitsConfigs.SupplyCurrentLimit = config.MAX_SUPPLY_CURRENT;
        _currentLimitsConfigs.StatorCurrentLimitEnable = config.ENABLE_STATOR_CURRENT_LIMIT;
        _currentLimitsConfigs.SupplyCurrentLimitEnable = config.ENABLE_SUPPLY_CURRENT_LIMIT;

        _torqueCurrentConfigs.PeakForwardTorqueCurrent = config.MAX_CURRENT;
        _torqueCurrentConfigs.PeakReverseTorqueCurrent = -config.MAX_CURRENT;
        _torqueCurrentConfigs.TorqueNeutralDeadband = config.CURRENT_DEADBAND;

        _voltageConfigs.PeakForwardVoltage = config.MAX_VOLTAGE;
        _voltageConfigs.PeakReverseVoltage = -config.MAX_VOLTAGE;
        _voltageConfigs.SupplyVoltageTimeConstant = config.VOLTAGE_TIME_CONSTANT;

        _feedbackConfigs.FeedbackRemoteSensorID = config.SENSOR_ID;
        _feedbackConfigs.FeedbackSensorSource = config.FEEDBACK_SENSOR;
        _feedbackConfigs.SensorToMechanismRatio = config.SENSOR_TO_MECHANISM_RATIO;

        _voltageOut.EnableFOC = config.USE_FOC;
        _motionMagicVoltage.EnableFOC = config.USE_FOC;
        _percentOutput.EnableFOC = config.USE_FOC;
        _velocityVoltage.EnableFOC = config.USE_FOC;
        _torqueCurrentFOC.Deadband = config.CURRENT_DEADBAND;

        _configurator.apply(_motorConfigs, config.TIME_OUT);
        _configurator.apply(_torqueCurrentConfigs, config.TIME_OUT);
        _configurator.apply(_currentLimitsConfigs, config.TIME_OUT);
        _configurator.apply(_feedbackConfigs, config.TIME_OUT);
    }

    public void configureFeedback(FeedbackConfigs config) {
        _configurator.apply(config, 100);
    }

    public void configureClosedLoop(ClosedLoopConfiguration config) {
        _slot0Configs.kV = config.kF;
        _slot0Configs.kP = config.kP;
        _slot0Configs.kI = config.kI;
        _slot0Configs.kD = config.kD; // Defaults to config 0
        _slot1Configs.kV = config.kF1;
        _slot1Configs.kP = config.kP1;
        _slot1Configs.kI = config.kI1;
        _slot1Configs.kD = config.kD1;

        _motionMagicConfigs.MotionMagicCruiseVelocity = config.CRUISE_VELOCITY;
        _motionMagicConfigs.MotionMagicAcceleration = config.ACCELERATION;
        _motionMagicConfigs.MotionMagicJerk = config.JERK;

        _closedLoopGenConfig.ContinuousWrap = config.IS_CONTINUOUS;

        _configurator.apply(_closedLoopGenConfig);
        _configurator.apply(_slot0Configs, config.TIME_OUT);
        _configurator.apply(_slot1Configs, config.TIME_OUT);
        _configurator.apply(_motionMagicConfigs);
    }

    public static double ticksToRadians(double ticks, double gearRatio) {
        return ticks * ((2.0 * Math.PI) / (gearRatio * 2048.0));
    }

    public static double radiansToRotations(double radians, double gearRatio) {
        return radians / ((2.0 * Math.PI) / gearRatio);
    }

    public static double radiansToTicks(double degrees, double gearRatio) {
        return degrees / ((2.0 * Math.PI) / (gearRatio * 2048.0));
    }

    public static double rotationsToRadians(double rotations, double gearRatio) {
        return rotations * ((2.0 * Math.PI) / gearRatio);
    }

    public static double ticksPer100msToRPM(double velocityCounts, double gearRatio) {
        double RPM = velocityCounts * (600.0 / 2048.0);
        return RPM / gearRatio;
    }

    public static double rotationsPerSecToRPM(double velocity, double gearRatio) {
        double RPM = velocity * (60.0);
        return RPM / gearRatio;
    }

    public static double RPMToTicksPer100ms(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        return motorRPM * (2048.0 / 600.0);
    }

    public static class ClosedLoopConfiguration {
        public double TIME_OUT = 0.1;
        public int SLOT = 0;

        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.0;
        public double MAX_INTEGRAL_ACCUMULATOR = 0;
        public int I_ZONE = 0; // Ticks
        public int TOLERANCE = 0; // Ticks

        public double RAMP_RATE = 0.0;

        public int CRUISE_VELOCITY = 0; // RPS
        public int ACCELERATION = 0; // RPS / Second
        public int JERK = 0; // RPS / Second / Second

        public double kP1 = 0.0;
        public double kI1 = 0.0;
        public double kD1 = 0.0;
        public double kF1 = 0.0;

        public boolean IS_CONTINUOUS = false;
    }

    public static class Configuration {
        public double TIME_OUT = 0.1; // seconds
        // motor configs
        public NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        public InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
        public boolean USE_FOC = false;

        // current/torque config
        public double MAX_CURRENT = 60.0;
        public double CURRENT_DEADBAND = 0.0;

        // voltage config
        public double MAX_VOLTAGE = 12.0;
        public double VOLTAGE_TIME_CONSTANT = 0.0;

        // current limits
        public double MAX_STATOR_CURRENT = 60.0;
        public double MAX_SUPPLY_CURRENT = 60.0;
        public boolean ENABLE_STATOR_CURRENT_LIMIT = false;
        public boolean ENABLE_SUPPLY_CURRENT_LIMIT = false;

        // feedback
        public int SENSOR_ID = 0;
        public FeedbackSensorSourceValue FEEDBACK_SENSOR = FeedbackSensorSourceValue.RotorSensor;
        public double SENSOR_TO_MECHANISM_RATIO = 1.0;
    }
}
