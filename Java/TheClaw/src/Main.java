import java.util.Scanner;
import java.util.UUID;

import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.ThreadContext;

public class Main {

    private static final Logit logger = Logit.getLogit("Main");
    private static SerialThread serialThread;
    private static UUID contextId;

    public static void main(String[] args) {
        contextId = UUID.randomUUID();

        serialThread = new SerialThread();
        serialThread.start();

        logger.log(Level.INFO, "Started");
        logger.log(Level.INFO, "Press <Enter> at the prompt to quit.");
        boolean running = true;
        while (running && !serialThread.isClosed()) {
            logger.log(Level.INFO, "What angle for forearm?");
            Scanner input = new Scanner(System.in);
            String in = input.nextLine();
            if (in.isEmpty()) {
                running = false;
            } else if (in.equalsIgnoreCase("burst")) {
                for (int i = 0; i < 180; i += 1) {
                    sendCommand(Stepper.FORE_ARM, (float) i);
                }
            } else {
                float angle = Float.parseFloat(in);
                sendCommand(Stepper.FORE_ARM, angle);
            }
        }

        int qSize = 0;
        int prevQSize;
        do {
            prevQSize = qSize;
            qSize = serialThread.getQueueSize();
            if (qSize > 0 && qSize != prevQSize) {
                logger.log(Level.INFO, "Waiting for command Q to empty: " + qSize);
            }
        } while (qSize > 0);
        logger.log(Level.INFO, "Closing");
        serialThread.close();
    }

    private static void sendCommand(Stepper motor, float angle) {
        ThreadContext.push(contextId.toString());
        logger.log(Level.DEBUG, "Sending <" + motor + ", " + angle + ">");
        serialThread.sendCommand(motor, angle);
        ThreadContext.pop();
    }
}
