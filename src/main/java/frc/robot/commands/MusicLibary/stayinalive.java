package frc.robot.commands.MusicLibary;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MusicMaker;

public class stayinalive extends CommandBase{
    private MusicMaker music;

    public stayinalive(MusicMaker maker){
        this.music = maker;


    }
    public void execute(){
        music.LoadSong("stayinalive");
        music.Play(true);
    }
    
    
    
}
