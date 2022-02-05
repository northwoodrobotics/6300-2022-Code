package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class Playmusic extends CommandBase{
    private final ShooterSubsystem subsystem;
    public Playmusic(ShooterSubsystem m_subsystem){
        subsystem = m_subsystem;

    }
    @Override
    public void execute(){
        subsystem.playmusic();
    }
    
}
