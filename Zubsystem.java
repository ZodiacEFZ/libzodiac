package frc.libzodiac;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ui.Xbox;

public abstract class Zubsystem extends SubsystemBase {
    public Zubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public final void periodic() {
        this.update();
    }

    protected Zubsystem update() {
        return this;
    }
}
