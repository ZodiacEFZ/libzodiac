package frc.libzodiac;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Allow you to construct <i>WPILib</i>'s <code>Command</code> with a lambda.
 */
public final class Zambda extends ZCommand {

    /**
     * The action to perform.
     */
    private final Runnable exec;

    /**
     * Create a command from code with specified requirement(s).
     * 
     * @param req  requirement of this command
     * @param exec the code to execute
     */
    public Zambda(Subsystem req, Runnable exec) {
        this.exec = exec;
        this.require(req);
    }

    /**
     * Create a command from code with specified requirement(s).
     * 
     * @param req  requirement of this command
     * @param exec the code to execute
     */
    public Zambda(Subsystem[] req, Runnable exec) {
        this.exec = exec;
        for (final var i : req) {
            this.require(i);
        }
    }

    /**
     * Create an independant/no requirement command.
     * 
     * @param exec the code to execute
     * @return a new command
     */
    public static Zambda indep(Runnable exec) {
        return new Zambda(new Subsystem[] {}, exec);
    }

    @Override
    protected ZCommand exec() {
        this.exec.run();
        return this;
    }
}