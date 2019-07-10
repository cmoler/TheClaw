package claw.Serial;

public class AtomicMotorCommand {

    public MotorCommand[] commands;

    public AtomicMotorCommand(MotorCommand ...commands) {
        this.commands = commands;
    }
}
