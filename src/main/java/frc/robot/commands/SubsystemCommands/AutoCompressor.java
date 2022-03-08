package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoCompressor extends CommandBase{
    private Compressor compressor;
    
    public AutoCompressor(Compressor subsystem){
        this.compressor = subsystem;
    }
    @Override
    public void initialize() {
        compressor.enableDigital();
    }
    
    @Override
    public void execute(){
        if(DriverStation.isAutonomous() ||RobotController.getBatteryVoltage() <= 8.5) {
            compressor.disable();
        }
        else {
            compressor.enabled();
        }
        
    }
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
