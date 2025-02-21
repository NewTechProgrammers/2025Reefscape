package frc.robot.commands.inTake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

public class IntakeTakeCommand extends Command{
    private final Intake intake;

    public IntakeTakeCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {       
    }

    @Override
    public void execute() {
        intake.take();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(intake);
    }
}
