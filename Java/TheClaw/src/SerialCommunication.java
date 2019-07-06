import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;
import org.apache.logging.log4j.Level;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Enumeration;

public class SerialCommunication implements SerialPortEventListener{

    SerialPort serialPort;

    private Logit logger = Logit.getLogit("SerialCom");

    /**
     * A BufferedReader which will be fed by a InputStreamReader
     * converting the bytes into characters
     * making the displayed results codepage independent
     */
    private BufferedReader input;
    /** The output stream to the port */
    private OutputStream output;
    /** Milliseconds to block while waiting for port open */
    private static final int TIME_OUT = 2000;
    /** Default bits per second for COM port. */
    private static final int DATA_RATE = 9600;

    public void initialize(String[] portNames) throws IllegalStateException {
        CommPortIdentifier portId = null;
        Enumeration portEnum = CommPortIdentifier.getPortIdentifiers();

        //First, Find an instance of serial port as set in PORT_NAMES.
        while (portEnum.hasMoreElements()) {
            CommPortIdentifier currPortId = (CommPortIdentifier) portEnum.nextElement();
            logger.log(Level.INFO,"Found port " + currPortId.getName());
            for (String portName : portNames) {
                if (currPortId.getName().equals(portName)) {
                    portId = currPortId;
                    break;
                }
            }
        }
        if (portId == null) {
            throw new IllegalStateException("Could not find COM port.");
        }

        try {
            // open serial port, and use class name for the appName.
            serialPort = (SerialPort) portId.open(this.getClass().getName(),
                    TIME_OUT);

            // set port parameters
            serialPort.setSerialPortParams(DATA_RATE,
                    SerialPort.DATABITS_8,
                    SerialPort.STOPBITS_1,
                    SerialPort.PARITY_NONE);

            // open the streams
            input = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
            output = serialPort.getOutputStream();

            // add event listeners
            serialPort.addEventListener(this);
            serialPort.notifyOnDataAvailable(true);
        } catch (Exception e) {
            logger.log(Level.ERROR,e.toString());
        }
    }

    /**
     * This should be called when you stop using the port.
     * This will prevent port locking on platforms like Linux.
     */
    public synchronized void close() {
        if (serialPort != null) {
            serialPort.removeEventListener();
            serialPort.close();
        }
    }

    public synchronized void serialEventOut(Stepper motor, int steps){
        byte[] stepsArray = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(steps).array();

        byte[] out = new byte[8];
        out[0] = '<';
        out[1] = (byte)motor.getValue();
        out[2] = ',';
        out[3] = stepsArray[0];
        out[4] = stepsArray[1];
        out[5] = stepsArray[2];
        out[6] = stepsArray[3];
        out[7] = '>';

        try {
            output.write(out);
        } catch (IOException e) {
            logger.log(Level.ERROR, e.toString());
        }
    }

    /**
     * Handle an event on the serial port. Read the data and print it.
     */
    public synchronized void serialEvent(SerialPortEvent oEvent) {
        if (oEvent.getEventType() == SerialPortEvent.DATA_AVAILABLE) {
            try {
                if (input.ready()) {
                    String inputLine = input.readLine();
                    logger.log(Level.DEBUG, inputLine);
                }
            } catch (Exception e) {
                logger.log(Level.ERROR, e.toString());
            }
        }
        // Ignore all the other eventTypes, but you should consider the other ones.
    }

}
