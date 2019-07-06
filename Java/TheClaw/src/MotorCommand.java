public class MotorCommand {
    public Stepper motor;
    public int steps;

    public MotorCommand(Stepper motor, int steps) {
        this.motor = motor;
        this.steps = steps;
    }
}
