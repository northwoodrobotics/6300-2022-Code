package frc.robot.commands.BlindLightCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;

public class LimelightSwitchLEDMode extends CommandBase {
    /**
     * Creates a new LimelightSwitchPipeline.
     */
    public Vision.LEDMode mode;

    /**
     * Command to change the Limelight's current LEDMode
     * @param mode The LEDMode to change to
     */
    public LimelightSwitchLEDMode(Vision.LEDMode mode) {
        addRequirements(RobotContainer.blindlight);
        this.mode = mode;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.blindlight.setLEDMode(mode);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}