package claw.Serial;

import claw.Logit;
import org.apache.logging.log4j.Level;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class SerialThread extends Thread {

    private Logit logger = Logit.getLogit("SerialTh");
    private SerialCommunication serialCom;
    private boolean closed = false;
    private BlockingQueue<AtomicMotorCommand> commandQ;

    private static final String PORT_NAMES[] = {
//            "/dev/tty.usbserial-A9007UX1", // Mac OS X
//            "/dev/ttyUSB0", // Linux
            "COM4", // Windows
    };

    private static final int MAX_STEP = 1024;
    private static final long TTL_MS = 10;

    public SerialThread() {
        this.serialCom = new SerialCommunication();
        this.commandQ = new LinkedBlockingQueue<>(5);
    }

    public SerialCommunication getSerialCom() {
        return this.serialCom;
    }

    public boolean isClosed() {
        return this.closed;
    }

    public int getQueueSize() {
        return this.commandQ.size();
    }

    public MotorCommand parseCommand(Stepper motor, float angle) {
        return new MotorCommand(motor, this.getStepsFromAngle(angle));
    }

    public synchronized void sendCommand(Stepper motor, float angle) {
        if (this.closed) {
            return;
        }
        this.sendAtomicCommand(parseCommand(motor, angle));
    }

    public synchronized void sendAtomicCommand(MotorCommand ...commands) {
        commandQ.add(new AtomicMotorCommand(TTL_MS, commands));
    }

    private int getStepsFromAngle(float angle) {
        float boundedAngle = angle;
        while (boundedAngle < 0) {
            boundedAngle += 360f;
        }
        if (boundedAngle > 360) {
            boundedAngle = boundedAngle % 360;
        }
        return Math.round((boundedAngle / 360f) * (float)(MAX_STEP));
    }

    public synchronized void close() {
        if (!this.closed) {
            this.closed = true;
            this.serialCom.close();
        }
    }

    @Override
    public void run() {
        try {
            this.serialCom.initialize(PORT_NAMES);
        } catch (IllegalStateException e) {
            logger.log(Level.ERROR, e.toString());
            this.closed = true;
        }
        while (!this.closed) {
            if (commandQ.peek() != null) {
                AtomicMotorCommand atomicCmd = commandQ.remove();
                if (atomicCmd.isFresh()) {
                    int dataLen = 5;
                    int dataNum = atomicCmd.commands.length * dataLen;
                    int byteNum = 3 + dataNum;
                    byte[] bytes = new byte[byteNum];
                    int b = 0;
                    bytes[b++] = '<';
                    bytes[b++] = (byte)(dataNum / dataLen);
                    for (MotorCommand cmd : atomicCmd.commands) {
                        byte[] thisOut = this.serialCom.buildOutput(cmd.motor, cmd.steps);
                        for (int i = 0; i < thisOut.length; i++) {
                            bytes[b++] = thisOut[i];
                        }
                    }
                    bytes[b] = '>';
                    this.serialCom.serialEventOut(bytes);
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        logger.log(Level.ERROR, e.toString());
                    }
                }
            }
        }
    }
}
