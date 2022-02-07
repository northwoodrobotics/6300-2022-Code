package frc.robot.commands.MusicLibary;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class PlaySelectedSong extends CommandBase{
    private ShooterSubsystem music;

    public PlaySelectedSong(ShooterSubsystem maker){
        this.music = maker;


    }
    public void execute(){
        music.LoadMusic();
        music.PlayMusic();
    }
    


    
}
