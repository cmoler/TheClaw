package claw;

import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.ThreadContext;

import java.util.UUID;

public class Logit {
    private Logger logger;
    private UUID contextId;

    public Logit(Logger logger) {
        this.logger = logger;
        this.contextId = UUID.randomUUID();
    }

    public UUID getContextId() {
        return this.contextId;
    }

    public void log(Level level, String ...messages) {
        ThreadContext.push(this.contextId.toString());
        for (String message : messages) {
            this.logger.log(level, message);
        }
        ThreadContext.pop();
    }

    public static Logit getLogit(String name) {
        return new Logit(LogManager.getLogger(name));
    }
}
