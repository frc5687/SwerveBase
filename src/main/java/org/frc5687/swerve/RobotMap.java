/* Team 5687 (C)2020-2021 */
package org.frc5687.swerve;

public class RobotMap {

    /**
     * There should be an entry here for each CAN device, preferrably grouped by device type and
     * then in numerical order. Note that for CAN, ids must be unique per device type, but not
     * across types. Thus, you cannot have two SparkMax controllers with Id 0, but you can have a
     * SparkMax with Id 0 and a TalonSRX with Id 0.
     */
    public static class CAN {

        public static class TALONFX {
            public static final int SW_RIGHT_FALCON = 3; // real 3
            public static final int SW_LEFT_FALCON = 4; // real 4
            public static final int SE_RIGHT_FALCON = 1;
            public static final int SE_LEFT_FALCON = 2;
            public static final int NW_RIGHT_FALCON = 5; // real 5
            public static final int NW_LEFT_FALCON = 6; // real 6
            public static final int NE_RIGHT_FALCON = 7;
            public static final int NE_LEFT_FALCON = 8;
        }
    }

    /**
     * There should be an entry here for each PWM port, preferrably in numerical order. Note that
     * for PWM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PWM {}

    /**
     * There should be an entry here for each PCM port, preferrably in numerical order. Note that
     * for PCM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PCM {}

    /**
     * There should be an entry here for each PDP breaker, preferrably in numerical order. Note that
     * only on device can be connected to each breaker, so the numbers should be unique.
     */
    public static class PDP {}

    /**
     * There should be an entry here for each Analgo port, preferrably in numerical order. Note that
     * for Analog only one device can connect to each port, so the numbers should be unique.
     */
    public static class Analog {}

    /**
     * There should be an entry here for each DIO port, preferrably in numerical order. Note that
     * for DIO only one device can connect to each port, so the numbers should be unique.
     */
    public static class DIO {
        public static final int ENCODER_NE = 4;
        public static final int ENCODER_NW = 3;
        public static final int ENCODER_SE = 5;
        public static final int ENCODER_SW = 2;
    }
}
