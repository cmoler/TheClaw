package claw.Serial;

import claw.Logit;
import org.apache.logging.log4j.Level;

import java.util.Scanner;

public class SerialMain {
    private static final Logit logger = Logit.getLogit("ArduinoMain");
    private static claw.Serial.SerialThread serialThread;

    public static void main(String[] args) {

        serialThread = new claw.Serial.SerialThread();
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
                    sendCommand(claw.Serial.Stepper.FORE_ARM, (float) i);
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

    private static void sendCommand(claw.Serial.Stepper motor, float angle) {
        logger.log(Level.DEBUG, "Sending <" + motor + ", " + angle + ">");
        serialThread.sendCommand(motor, angle);
    }
}
