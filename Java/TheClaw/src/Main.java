import java.io.IOException;

import claw.Leap.LeapListener;
import claw.Logit;
import claw.Serial.SerialThread;
import com.leapmotion.leap.Controller;

public class Main {

    private static final Logit logger = Logit.getLogit("Main");
    private static SerialThread serialThread;

    public static void main(String[] args) {
        serialThread = new SerialThread();
        serialThread.start();

        // claw.Leap Motion stuff
        LeapListener listener = new LeapListener(serialThread);
        Controller controller = new Controller();

        controller.addListener(listener);

        System.out.println("Press Enter to quit...");
        try {
            System.in.read();
        } catch (IOException e) {
            e.printStackTrace();
        }

        controller.removeListener(listener);
    }
}