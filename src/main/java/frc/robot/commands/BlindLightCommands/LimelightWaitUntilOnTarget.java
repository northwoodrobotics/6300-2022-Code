package frc.robot.commands.BlindLightCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class LimelightWaitUntilOnTarget extends CommandBase {
    public LimelightWaitUntilOnTarget() {
        addRequirements(RobotContainer.blindlight);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(RobotContainer.blindlight.getTargetAngleX()) <= 1;
    }
}
