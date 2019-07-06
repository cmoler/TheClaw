import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class SerialThread extends Thread {

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
        this.serialCom.initialize(PORT_NAMES);
        while (!this.closed) {
            if (commandQ.peek() != null) {
                MotorCommand cmd = commandQ.remove();
                System.out.println("Sending " + cmd.steps + " / " + MAX_STEP);
                this.serialCom.serialEventOut(cmd.motor, cmd.steps);
            }
        }
    }
}
