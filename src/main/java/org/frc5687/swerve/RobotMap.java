/* Team 5687 (C)2020-2021 */
package org.frc5687.swerve;


public class RobotMap {

    /**
     * There should be an entry here for each CAN device, preferrably grouped by device type and then
     * in numerical order. Note that for CAN, ids must be unique per device type, but not across
     * types. Thus, you cannot have two SparkMax controllers with Id 0, but you can have a SparkMax
     * with Id 0 and a TalonSRX with Id 0.
     */
    public static class CAN {

        public static class TALONFX {
            public static final int NORTH_WEST_OUTER = 30;
            public static final int NORTH_WEST_INNER = 31;
            public static final int NORTH_EAST_INNER = 5;
            public static final int NORTH_EAST_OUTER = 6;
            public static final int SOUTH_EAST_OUTER = 32;
            public static final int SOUTH_EAST_INNER = 33;
            public static final int SOUTH_WEST_INNER = 34;
            public static final int SOUTH_WEST_OUTER = 35;
            public static final int ARM = 9;
            public static final int EXT_ARM = 12;
            public static final int CUBESHOOTER_WRIST = 15;
            public static final int CUBESHOOTER_SHOOTER = 14;
        }

        public static class PIGEON {
            public static final int PIGEON = 0;
        }

        public static class CANDLE {
            public static final int PORT = 13;
        }

        public static class TalonSRX {
            public static final int GRIPPER = 11;
            public static final int WRIST = 10;
        }
    }

    /**
     * There should be an entry here for each PWM port, preferrably in numerical order. Note that for
     * PWM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PWM {
        public static class Servo{
            public static final int NE_SERVO = 0;
            public static final int SE_SERVO = 1;
            public static final int NW_SERVO = 2;
            public static final int SW_SERVO = 3;
        }
    }

    /**
     * There should be an entry here for each PCM port, preferrably in numerical order. Note that for
     * PCM only one device can connect to each port, so the numbers should be unique.
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
     * There should be an entry here for each DIO port, preferrably in numerical order. Note that for
     * DIO only one device can connect to each port, so the numbers should be unique.
     */
    public static class DIO {
        public static final int ENCODER_NE = 0; // takes up 3 slots ABS, A, B channels (ie 1 and 2 are also used).
        public static final int ENCODER_NW = 10; // takes up 3 slots ABS, A, B channels.
        public static final int ENCODER_SE = 16; // takes up 3 slots ABS, A, B channels.
        public static final int ENCODER_SW = 13; // takes up 3 slots ABS, A, B channels.

        public static final int ENCODER_GRIPPER = 4;
        public static final int ENCODER_WRIST = 5;
        // public static final int TOP_HALL_ARM = 9; // north is 9
        //        public static final int BOTTOM_HALL_ARM = 8; // south is 8

        public static final int IN_EXT_HALL = 6;
        public static final int CUBESHOOTER_PROXIMITY = 7;
        public static final int ARM_ENCODER = 8;
        public static final int ENCODER_CUBESHOOTER_WRIST = 9;
        public static final int ARM_ENCODER_A = 19;
        public static final int ARM_ENCODER_B = 20;
    }
}
