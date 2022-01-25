package frc.robot.commands.BlindLightCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class LimelightSwitchPipeline extends CommandBase {
    public int id;

    /**
     * Command to change the Limelight's current pipeline
     * @param id The pipeline id to change to
     */
    public LimelightSwitchPipeline(int id) {
        addRequirements(RobotContainer.blindlight);
        this.id = id;
    }

    @Override
    public void initialize() {
        RobotContainer.blindlight.setPipeline(id);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}