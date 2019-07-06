import java.util.Scanner;

public class Main {

    public static void main(String[] args) {
        SerialThread serialThread = new SerialThread();
        serialThread.start();

        System.out.println("Started");
        boolean running = true;
        while (running) {
            System.out.println("What angle for forearm?");
            Scanner input = new Scanner(System.in);
            String in = input.nextLine();
            if (in.isEmpty()) {
                running = false;
            } else if (in.equalsIgnoreCase("burst")) {
                for (int i = 0; i < 180; i += 1) {
                    serialThread.sendCommand(Stepper.FORE_ARM, (float) i);
                }
            } else {
                float angle = Float.parseFloat(in);
                serialThread.sendCommand(Stepper.FORE_ARM, angle);
            }
        }
        while (serialThread.getQueueSize() > 0) {}
        System.out.println("Closing");
        serialThread.close();
    }
}
