import org.apache.logging.log4j.Level;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class SerialThread extends Thread {

    private Logit logger = Logit.getLogit("SerialTh");
    private SerialCommunication serialCom;
    private boolean closed = false;
    private BlockingQueue<MotorCommand> commandQ;

    private static final String PORT_NAMES[] = {
//            "/dev/tty.usbserial-A9007UX1", // Mac OS X
//            "/dev/ttyUSB0", // Linux
            "COM4", // Windows
    };

    private static final int MAX_STEP = 1024;

    public SerialThread() {
        this.serialCom = new SerialCommunication();
        this.commandQ = new LinkedBlockingQueue<>();
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

    public synchronized void sendCommand(Stepper motor, float angle) {
        if (this.closed) {
            return;
        }
        float boundedAngle = angle;
        while (boundedAngle < 0) {
            boundedAngle += 360f;
        }
        boundedAngle = Math.min(Math.max(boundedAngle % 360, 15), 165);
        int steps = Math.round((boundedAngle / 360f) * (float)(MAX_STEP));
        commandQ.add(new MotorCommand(motor, steps));
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
                MotorCommand cmd = commandQ.remove();
                logger.log(Level.DEBUG, "Sending " + cmd.steps + " / " + MAX_STEP);
                this.serialCom.serialEventOut(cmd.motor, cmd.steps);
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    logger.log(Level.ERROR, e.toString());
                }
            }
        }
    }
}
