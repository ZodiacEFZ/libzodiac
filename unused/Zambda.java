package frc.libzodiac.unused;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Allow you to construct <i>WPILib</i>'s <code>Command</code> with a lambda.
 */
public final class Zambda extends Command {

    /**
     * The action to perform.
     */
    private Runnable exec;

    private Zambda(Subsystem... dep) {
        this.addRequirements(dep);
    }

    /**
     * Create a command with specified dependencies. By default the command runs
     * nothing unless configured with <code>.runs()</code>.
     *
     * @param dep dependencies of the command
     * @return new
     */
    public static Zambda req(Subsystem... dep) {
        return new Zambda(dep);
    }

    /**
     * Specify what action to perform with the command.
     *
     * @param exec the code to run
     * @return self for chaining
     */
    public Zambda runs(Runnable exec) {
        this.exec = exec;
        return this;
    }

    @Override
    public void execute() {
        this.exec.run();
    }
}