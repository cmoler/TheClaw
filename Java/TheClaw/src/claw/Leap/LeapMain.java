package claw.Leap;

import claw.Logit;
import com.leapmotion.leap.Controller;

import java.io.IOException;

public class LeapMain {
    private static final Logit logger = Logit.getLogit("claw.Leap.LeapMain");

    public static void main(String[] args) {
        // claw.Leap Motion stuff
        LeapListener listener = new LeapListener();
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
