public class Main {

    public static void main(String[] args) {
        SerialCommunication serialCommunication = new SerialCommunication();
        serialCommunication.initialize();
        Thread t = new Thread(){
          public void run() {
              try { Thread.sleep(1000000); } catch (InterruptedException ie) {}
          }
        };

        t.start();
        System.out.println("Started");
    }
}
