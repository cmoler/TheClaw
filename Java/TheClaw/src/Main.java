import java.util.Scanner;

public class Main {

    public static void main(String[] args) {
        SerialCommunication serialCommunication = new SerialCommunication();
        serialCommunication.initialize();
        Thread t = new Thread(){
          public void run() {
              try {
                  Thread.sleep(10);
                  Scanner input = new Scanner(System.in);
                  int steps = input.nextInt();
                  serialCommunication.serialEventOut(Stepper.FORE_ARM, steps);
              } catch (InterruptedException ie) {}
          }
        };
        t.start();




        System.out.println("Started");
    }
}
