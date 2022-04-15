/* Team 5687 (C)2022 */
package org.frc5687.swerve.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import org.frc5687.swerve.util.OutlierPeriodic;
import org.zeromq.SocketType;
import org.zeromq.ZContext;
import org.zeromq.ZMQ;

public class Coprocessor implements OutlierPeriodic {

    private static final String SOCKET = "tcp://10.56.87.20:27002";
    private ZContext _context;
    private ZMQ.Socket _socket;

    private JetsonFrames.ReceiveFrame _latestFrame;

    public Coprocessor() {
        try {
            _context = new ZContext();
            _socket = _context.createSocket(SocketType.REQ);
            _socket.connect(SOCKET);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), false);
        }
    }

    @Override
    public void controlPeriodic(double timestamp) {}

    @Override
    public void dataPeriodic(double timestamp) {
        receive();
    }

    public synchronized void receive() {
        // read data
        String incomingData = new String(_socket.recv(0), ZMQ.CHARSET);
        JetsonFrames.ReceiveFrame incomingFrame = new JetsonFrames.ReceiveFrame(incomingData);

        // check if latest frame
        if (_latestFrame.getMillis() < incomingFrame.getMillis()) {
            _latestFrame = incomingFrame;
        }
    }

    public synchronized void sendMessage(JetsonFrames.SendFrame frame) {
        _socket.send(frame.messagePacket().getBytes(ZMQ.CHARSET));
    }

    public static class JetsonFrames {
        public static class SendFrame {
            private final long _millis;
            private final List<Data> _data = new ArrayList<>();

            public SendFrame(Data... data) {
                _millis = System.currentTimeMillis();
                _data.addAll(List.of(data));
            }

            public String messagePacket() {
                StringBuilder buffer = new StringBuilder();

                buffer.append(_millis);
                buffer.append(";");

                for (Data d : _data) {
                    buffer.append(d.toString());
                    buffer.append(";");
                }
                return buffer.toString();
            }
        }

        static class ReceiveFrame {
            private final long _millis;
            private final double _estimatedX;
            private final double _estimatedY;
            private final double _estimatedHeading;
            private final boolean _hasTarget;
            private final double _goalDistance;
            private final double _goalAngle;

            public ReceiveFrame(String packet) {
                String[] a = packet.split(";");
                _millis = Long.parseLong(a[0]);
                _estimatedX = Double.parseDouble(a[1]);
                _estimatedY = Double.parseDouble(a[2]);
                _estimatedHeading = Double.parseDouble(a[3]);
                _hasTarget = Boolean.parseBoolean(a[4]);
                _goalDistance = Double.parseDouble(a[5]);
                _goalAngle = Double.parseDouble(a[6]);
            }

            public long getMillis() {
                return _millis;
            }

            public Pose2d getEstimatedPose() {
                return new Pose2d(_estimatedX, _estimatedY, new Rotation2d(_estimatedHeading));
            }

            public double getTargetDistance() {
                return _goalDistance;
            }

            public double getTargetAngle() {
                return _goalAngle;
            }

            public boolean hadTarget() {
                return _hasTarget;
            }
        }

        public static class Data {
            private Double _double = null;
            private Long _long = null;
            private String _string = null;
            private Boolean _boolean = null;
            private Integer _int = null;

            public Data(double data) {
                _double = data;
            }

            public Data(long data) {
                _long = data;
            }

            public Data(String data) {
                _string = data;
            }

            public Data(Boolean data) {
                _boolean = data;
            }

            public Data(int data) {
                _int = data;
            }

            @Override
            public String toString() {
                if (_double != null) {
                    return _double.toString();
                } else if (_long != null) {
                    return _long.toString();
                } else if (_string != null) {
                    return _string;
                } else if (_boolean != null) {
                    return _boolean.toString();
                } else if (_int != null) {
                    return _int.toString();
                }
                return null;
            }
        }
    }
}
