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
    static final double LOWER_ARM_ANGLE_MIN = Math.toRadians(15); // 0.349
    static final double LOWER_ARM_ANGLE_MAX = Math.toRadians(90); // 1.571
    static final double UPPER_ARM_ANGLE_MIN = Math.toRadians(35); // 0.698
    static final double UPPER_ARM_ANGLE_MAX = Math.toRadians(180); // 3.142
    static final double BASE_ANGLE_MIN = Math.toRadians(15);
    static final double BASE_ANGLE_MAX = Math.toRadians(165); // 2.618

    static final float Y_MIN = -10;
    static final float Y_MAX = TOTAL_ARM_LEN;
    //static final float X_MAX = (float) (Math.cos(LOWER_ARM_ANGLE_MIN) * TOTAL_ARM_LEN); // 195.461

    static final float FINGER_DIFF_MAX = 100;
    static final float FINGER_DIFF_MIN = 30;

    // Value of 100 would mean final range of 200mm across
    static final float SCALE_CONSTANT = 300;

    static final float LERP_GRIPPER_FACTOR = 0.5f;
    static final float LERP_GRIPPER_MIN_DIST = 0.05f;
    static final float LERP_HAND_FACTOR = 1f;
    static final float LERP_HAND_MIN_DIST = 3f;

    ArmCalculator armCalculator;

    Logit logger = Logit.getLogit("LeapListener");
    Vector mappedPosition = null;
    float gripperRatio = -1;
    LeapPosition leapPosition;
    SerialThread serialThread;
    int frames = -1;
    int frameDelay = 20;
    boolean hasNewValue = true;

    public LeapListener(SerialThread serialThread) {
        this.serialThread = serialThread;
        this.armCalculator = new ArmCalculator(
            LOWER_ARM_LEN,
            UPPER_ARM_LEN,
            GRIPPER_LEN
        );
        this.armCalculator.setBaseLimits(BASE_ANGLE_MIN, BASE_ANGLE_MAX);
        this.armCalculator.setLowerArmLimits(LOWER_ARM_ANGLE_MIN, LOWER_ARM_ANGLE_MAX);
        this.armCalculator.setUpperArmLimits(UPPER_ARM_ANGLE_MIN, UPPER_ARM_ANGLE_MAX);
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

        FingerList fingers = frame.fingers();
        FingerList indexFingers = fingers.fingerType(Finger.Type.TYPE_INDEX);
        FingerList thumbFingers = fingers.fingerType(Finger.Type.TYPE_THUMB);

        if (fingers.count() >= 2 && !indexFingers.isEmpty() && !thumbFingers.isEmpty()) {
            Finger indexFinger = indexFingers.get(0);
            Finger thumbFinger = thumbFingers.get(0);

            Vector thumbPos = thumbFinger.stabilizedTipPosition();
            Vector indexThumbDiff = indexFinger.stabilizedTipPosition().minus(thumbPos);

            // Vector inputPos = thumbPos;
            Vector inputPos = thumbPos.plus(indexThumbDiff.times(0.5f));

            // Scale from claw.Leap Space to Robo Arm Space
            Vector targetPosition = mapLeapToWorld(inputPos, interactionBox);
            if (mappedPosition == null) {
                mappedPosition = targetPosition;
            } else {
                Vector newPos = armCalculator.lerpVector(mappedPosition, targetPosition, LERP_HAND_FACTOR, LERP_HAND_MIN_DIST);
                if (newPos.minus(mappedPosition).magnitude() > LERP_HAND_MIN_DIST) {
                    hasNewValue = true;
                }
                mappedPosition = newPos;
            }

            // Calculate Angles
            float z = Math.max(mappedPosition.getZ() * -1, 0);
            double baseAngle = armCalculator.calcBaseAngle(mappedPosition.getX(), z);

            double xz = Math.hypot(mappedPosition.getX(), z);
            double y = Math.max(mappedPosition.getY(), Y_MIN);
            double[] kinematicsAngles = armCalculator.calcInverseKinematics(y, xz);

            //logger.log(Level.DEBUG, String.format("%f, %f", xz, y));

            // Get gripper close ratio, 0 for open hand - 1 for closed "pinch"
            float fingerDistCapped = Math.min(Math.max(indexThumbDiff.magnitude(), FINGER_DIFF_MIN), FINGER_DIFF_MAX);
            float targetGripperRatio =  1 - ((fingerDistCapped - FINGER_DIFF_MIN) / (FINGER_DIFF_MAX - FINGER_DIFF_MIN));
            if (gripperRatio < 0) {
                gripperRatio = targetGripperRatio;
            } else {
                float newRatio = armCalculator.lerpFloat(gripperRatio, targetGripperRatio, LERP_GRIPPER_FACTOR, LERP_GRIPPER_MIN_DIST);
                if (Math.abs(newRatio - gripperRatio) > LERP_GRIPPER_MIN_DIST) {
                    hasNewValue = true;
                }
                gripperRatio = newRatio;
            }

            leapPosition.updateAngles(
                    (float) Math.toDegrees(baseAngle),
                    (float) Math.toDegrees(kinematicsAngles[0]), // Lower arm angle
                    (float) Math.toDegrees(kinematicsAngles[1]), // Upper arm angle
                    gripperRatio);
        }

        frames = (frames + 1) % frameDelay;
        if (frames > 0) {
            return;
        }

        if (hasNewValue) {
            logger.log(Level.DEBUG, leapPosition.toString());
            sendCommands();
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
      normalized = normalized.plus(new Vector(-0.5f, -0.25f, -0.25f));

      // Scale up to robot space
      normalized = normalized.times(SCALE_CONSTANT);

      return normalized;
    }
}