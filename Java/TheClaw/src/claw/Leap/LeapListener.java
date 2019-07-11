package claw.Leap;

import claw.Logit;
import claw.Serial.MotorCommand;
import claw.Serial.SerialThread;
import claw.Serial.Stepper;
import com.leapmotion.leap.*;
import org.apache.logging.log4j.Level;

/*
    Listener for the claw.Leap in order to grab values from it once it is connected
 */
public class LeapListener extends Listener {

    /*
        Constants: Max & Min values for Length and Angles
     */
    static final int LOWER_ARM_LEN = 90;
    static final int UPPER_ARM_LEN = 50;
    static final int GRIPPER_LEN = 68;
    static final int TOTAL_ARM_LEN = LOWER_ARM_LEN + UPPER_ARM_LEN + GRIPPER_LEN; // 208

    // All angle calculations use radians, casted back to degrees before sent to arduino
    static final double LOWER_ARM_ANGLE_MIN = Math.toRadians(20); // 0.349
    static final double LOWER_ARM_ANGLE_MAX = Math.toRadians(90); // 1.571
    static final double UPPER_ARM_ANGLE_MIN = Math.toRadians(40); // 0.698
    static final double UPPER_ARM_ANGLE_MAX = Math.toRadians(180); // 3.142
    static final double BASE_ANGLE_MIN = Math.toRadians(0);
    static final double BASE_ANGLE_MAX = Math.toRadians(150); // 2.618

    static final float Y_MAX = TOTAL_ARM_LEN;
    static final float X_MAX = (float) (Math.cos(LOWER_ARM_ANGLE_MIN) * TOTAL_ARM_LEN); // 195.461

    // Inner hypotenuse minimum length calculated using law of cosines with min upper arm angle
    static final double INNER_HYPOT_MIN = Math.sqrt((LOWER_ARM_LEN * LOWER_ARM_LEN)
            + ((UPPER_ARM_LEN + GRIPPER_LEN) * (UPPER_ARM_LEN + GRIPPER_LEN))
          - (2 * LOWER_ARM_LEN * (UPPER_ARM_LEN + GRIPPER_LEN) * Math.cos(UPPER_ARM_ANGLE_MIN))); // 6.741

    // Value of 100 would mean final range of 200mm across
    static final float SCALE_CONSTANT = TOTAL_ARM_LEN;

    Logit logger = Logit.getLogit("LeapListener");
    LeapPosition leapPosition;
    SerialThread serialThread;
    int frames = -1;
    int frameDelay = 10;

    public LeapListener(SerialThread serialThread) {
        this.serialThread = serialThread;
    }

    public void onInit(Controller controller) {
        logger.log(Level.INFO, "Listener Initialized");
    }

    public void onConnect(Controller controller) {
        logger.log(Level.INFO, "Connected");
        leapPosition = new LeapPosition();
    }

    public void onDisconnect(Controller controller) {
        logger.log(Level.INFO,"Disconnected");
    }

    public void onExit(Controller controller) {
        if (serialThread != null) {
            int qSize = 0;
            int prevQSize;
            do {
                prevQSize = qSize;
                qSize = serialThread.getQueueSize();
                if (qSize > 0 && qSize != prevQSize) {
                    logger.log(Level.INFO, "Waiting for command Q to empty: " + qSize);
                }
            } while (qSize > 0);
            logger.log(Level.INFO, "Closing serial thread...");
            serialThread.close();
        }
        logger.log(Level.INFO, "Exited");
    }

    public void onFrame(Controller controller) {
        // logger.log(Level.INFO, "Frame available");
        Frame frame = controller.frame();
        InteractionBox interactionBox = frame.interactionBox();

        if (!frame.hands().isEmpty()) {
            Hand hand = frame.hands().get(0);
            Vector palmPosition = hand.palmPosition();

            // Scale from claw.Leap Space to Robo Arm Space
            Vector mappedPosition = mapLeapToWorld(palmPosition, interactionBox);

            // Calculate Angles
            double baseAngle = calcBaseAngle(mappedPosition.getX(), mappedPosition.getZ());

            double xz = Math.hypot(mappedPosition.getX(), mappedPosition.getZ());
            double y = Math.max(mappedPosition.getY(), 0);
            double[] kinematicsAngles = calcInverseKinematics(y, xz);

            // Get gripper close ratio, 0 for open hand - 1 for closed "pinch"
            float gripperRatio = hand.pinchStrength();

            // Set all angles as degrees within the claw.Leap.LeapPosition to send to arduino
            leapPosition.updateAngles(
                    (float) Math.toDegrees(baseAngle),
                    (float) Math.toDegrees(kinematicsAngles[0]), // Lower arm angle
                    (float) Math.toDegrees(kinematicsAngles[1]), // Upper arm angle
                    gripperRatio);


            // logg
            //
            // er.log(Level.DEBUG, leapPosition.toString());
        }

        frames = (frames + 1) % frameDelay;
        if (frames > 0) {
            return;
        }
        if (serialThread != null && !serialThread.isClosed()) {
            MotorCommand[] cmds = {
                    //serialThread.parseCommand(Stepper.BASE, leapPosition.baseAngle),
                    // serialThread.parseCommand(Stepper.LOWER_ARM, leapPosition.lowerArmAngle),
                    serialThread.parseCommand(Stepper.UPPER_ARM, leapPosition.upperArmAngle),
                    serialThread.parseCommand(Stepper.GRIPPER, leapPosition.gripperRatio * 360)
            };
            serialThread.sendAtomicCommand(cmds);
        }
    }

    private Vector mapLeapToWorld(Vector leapPoint, InteractionBox interactionBox){

      Vector normalized = interactionBox.normalizePoint(leapPoint, false);

      // Recenter origin from bottom back left to bottom center
      //normalized = normalized.plus(new Vector((float) 0.5, 0, (float) 0.5));

      // Scale up to robot space
      normalized = normalized.times(SCALE_CONSTANT);

      // Restrict max/min values
      if (normalized.getY() > Y_MAX) {
        normalized.setY(Y_MAX);
      }

      if (normalized.getX() > X_MAX) {
        normalized.setX(X_MAX);
      }

      if (normalized.getZ() > X_MAX) {
        normalized.setZ(X_MAX);
      }

      return normalized;
    }

    private double calcBaseAngle(float x, float z) {
        double angle = Math.tan(z / x);

        // Restrict angles to min and max
        if (angle > BASE_ANGLE_MAX) {
            angle = BASE_ANGLE_MAX;
        }
        else if (angle < BASE_ANGLE_MIN) {
            angle = BASE_ANGLE_MIN;
        }
        return angle;
    }

    private double[] calcInverseKinematics(double y, double xz) {
      double h = Math.hypot(y, xz);

      // Restrict hypotenuse to min and max robot arm range
      if (h > TOTAL_ARM_LEN) {
        h = TOTAL_ARM_LEN;
      } else if (h < INNER_HYPOT_MIN) {
        h = INNER_HYPOT_MIN;
      }

      // Lower arm angle is comprised of a1 (lower part using right triangle) and a2 (upper part using law of cosines)
      double a1 = Math.atan2(y, xz);

      double a2 = Math.acos(
              ((h * h) + (LOWER_ARM_LEN * LOWER_ARM_LEN)
                      - ((UPPER_ARM_LEN + GRIPPER_LEN) * (UPPER_ARM_LEN + GRIPPER_LEN)))
                      / (2 * h * LOWER_ARM_LEN));

      double lowerArmAngle = a1 + Math.abs(a2);
      lowerArmAngle = Math.min(Math.max(lowerArmAngle, LOWER_ARM_ANGLE_MIN), LOWER_ARM_ANGLE_MAX);

      // Upper arm angle calc using law of cosines
      double upperArmAngle = Math.acos(
              ((LOWER_ARM_LEN * LOWER_ARM_LEN)
                      + ((UPPER_ARM_LEN + GRIPPER_LEN) * (UPPER_ARM_LEN + GRIPPER_LEN))
                      - (h * h))
                      / (2 * LOWER_ARM_LEN * (UPPER_ARM_LEN + GRIPPER_LEN)));
      upperArmAngle = Math.min(Math.max(upperArmAngle, UPPER_ARM_ANGLE_MIN), UPPER_ARM_ANGLE_MAX);

      double[] angles = { lowerArmAngle, upperArmAngle };

      logger.log(Level.DEBUG, String.format("%f,%f,%f,%f,%f,%f,%f", xz, y, h, a1, a2, lowerArmAngle, upperArmAngle));

      return angles;
    }
}