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

    static final float Y_MIN = -10;
    static final float Y_MAX = TOTAL_ARM_LEN;
    static final float X_MAX = TOTAL_ARM_LEN;
    //static final float X_MAX = (float) (Math.cos(LOWER_ARM_ANGLE_MIN) * TOTAL_ARM_LEN); // 195.461

    static final float FINGER_DIFF_MAX = 100;
    static final float FINGER_DIFF_MIN = 20;

    // Inner hypotenuse minimum length calculated using law of cosines with min upper arm angle
    static final double INNER_HYPOT_MIN = Math.sqrt((LOWER_ARM_LEN * LOWER_ARM_LEN)
            + ((UPPER_ARM_LEN + GRIPPER_LEN) * (UPPER_ARM_LEN + GRIPPER_LEN))
          - (2 * LOWER_ARM_LEN * (UPPER_ARM_LEN + GRIPPER_LEN) * Math.cos(UPPER_ARM_ANGLE_MIN))); // 6.741

    // Value of 100 would mean final range of 200mm across
    static final float SCALE_CONSTANT = 300;

    static final float LERP_GRIPPER_FACTOR = 0.5f;
    static final float LERP_GRIPPER_MIN_DIST = 0.05f;
    static final float LERP_HAND_FACTOR = 1f;
    static final float LERP_HAND_MIN_DIST = 3f;

    Logit logger = Logit.getLogit("LeapListener");
    Vector mappedPosition = null;
    float gripperRatio = -1;
    LeapPosition leapPosition;
    SerialThread serialThread;
    int frames = -1;
    int frameDelay = 10;
    boolean hasNewValue = true;

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
        frames = (frames + 1) % frameDelay;
        if (frames > 0) {
            return;
        }

        Frame frame = controller.frame();
        InteractionBox interactionBox = frame.interactionBox();

        FingerList fingers = frame.fingers();
        FingerList indexFingers = fingers.fingerType(Finger.Type.TYPE_INDEX);
        FingerList thumbFingers = fingers.fingerType(Finger.Type.TYPE_THUMB);

        if (fingers.count() >= 2 && !indexFingers.isEmpty() && !thumbFingers.isEmpty()) {
            Finger indexFinger = indexFingers.get(0);
            Finger thumbFinger = thumbFingers.get(0);

            Vector thumbPos = thumbFinger.tipPosition();
            Vector indexThumbDiff = indexFinger.tipPosition().minus(thumbPos);

            // Vector inputPos = thumbPos;
            Vector inputPos = thumbPos.plus(indexThumbDiff.times(0.5f));

            // Scale from claw.Leap Space to Robo Arm Space
            Vector targetPosition = mapLeapToWorld(inputPos, interactionBox);
            if (mappedPosition == null) {
                mappedPosition = targetPosition;
            } else {
                Vector newPos = lerpVector(mappedPosition, targetPosition, LERP_HAND_FACTOR, LERP_HAND_MIN_DIST);
                if (newPos.minus(mappedPosition).magnitude() > LERP_HAND_MIN_DIST) {
                    hasNewValue = true;
                }
                mappedPosition = newPos;
            }

            // Calculate Angles
            // System.out.println("POS:" + mappedPosition.toString());
            float z = Math.max(mappedPosition.getZ() * -1, 0);
            double baseAngle = calcBaseAngle(mappedPosition.getX(), z);

            double xz = Math.hypot(mappedPosition.getX(), z);
            double y = Math.max(mappedPosition.getY(), Y_MIN);
            double[] kinematicsAngles = calcInverseKinematics(y, xz);

            // logger.log(Level.DEBUG, String.format("%f, %f", xz, y));

            // Get gripper close ratio, 0 for open hand - 1 for closed "pinch"
            float fingerDistCapped = Math.min(Math.max(indexThumbDiff.magnitude(), FINGER_DIFF_MIN), FINGER_DIFF_MAX);
            float targetGripperRatio =  1 - ((fingerDistCapped - FINGER_DIFF_MIN) / (FINGER_DIFF_MAX - FINGER_DIFF_MIN));
            // System.out.println(targetGripperRatio);
            if (gripperRatio < 0) {
                gripperRatio = targetGripperRatio;
            } else {
                float newRatio = lerpFloat(gripperRatio, targetGripperRatio, LERP_GRIPPER_FACTOR, LERP_GRIPPER_MIN_DIST);
                if (Math.abs(newRatio - gripperRatio) > LERP_GRIPPER_MIN_DIST) {
                    hasNewValue = true;
                }
                gripperRatio = newRatio;
            }

            if (hasNewValue) {
                // Set all angles as degrees within the claw.Leap.LeapPosition to send to arduino
                leapPosition.updateAngles(
                        (float) Math.toDegrees(baseAngle),
                        (float) Math.toDegrees(kinematicsAngles[0]), // Lower arm angle
                        (float) Math.toDegrees(kinematicsAngles[1]), // Upper arm angle
                        gripperRatio);

                // logger.log(Level.DEBUG, leapPosition.toString());

                sendCommands();
            }
        }
    }

    private void sendCommands() {
        if (serialThread != null && !serialThread.isClosed()) {
            MotorCommand[] cmds = {
                    serialThread.parseCommand(Stepper.BASE, leapPosition.baseAngle),
                    serialThread.parseCommand(Stepper.LOWER_ARM, leapPosition.lowerArmAngle),
                    serialThread.parseCommand(Stepper.UPPER_ARM, leapPosition.upperArmAngle),
                    serialThread.parseCommand(Stepper.GRIPPER, leapPosition.gripperRatio * 360)
            };
            serialThread.sendAtomicCommand(cmds);
        }
    }

    private Vector mapLeapToWorld(Vector leapPoint, InteractionBox interactionBox){

      Vector normalized = interactionBox.normalizePoint(leapPoint, false);

      // Recenter origin
      normalized = normalized.plus(new Vector((float) -0.5, (float)-0.25, (float) -0.25));

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
        double angle = Math.atan2(z, x);

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

      // logger.log(Level.DEBUG, String.format("%f,%f,%f,%f,%f,%f,%f", xz, y, h, a1, a2, lowerArmAngle, upperArmAngle));

      return angles;
    }

    private Vector lerpVector(Vector vecStart, Vector vecEnd, float lerpFactor, float minDist) {
        Vector vecOut;
        Vector diff = vecEnd.minus(vecStart);
        float dist = diff.magnitude();
        if (dist > minDist) {
            vecOut = vecStart.plus(diff.normalized().times(dist * lerpFactor));
        } else {
            vecOut = vecEnd;
        }
        return vecOut;
    }

    private float lerpFloat(float valStart, float valEnd, float lerpFactor, float minDist) {
        float out;
        float diff = valEnd - valStart;
        if (diff > minDist) {
            out = valStart + diff * lerpFactor;
        } else {
            out = valEnd;
        }
        return out;
    }
}