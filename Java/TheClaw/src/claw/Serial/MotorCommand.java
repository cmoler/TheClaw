package claw.Serial;

public class MotorCommand {
    public claw.Serial.Stepper motor;
    public int steps;

    public MotorCommand(claw.Serial.Stepper motor, int steps) {
        this.motor = motor;
        this.steps = steps;
    }
}
