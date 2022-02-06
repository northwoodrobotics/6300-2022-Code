package frc.robot.commands.MusicLibary;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MusicMaker;

public class rickroll extends CommandBase{
    private MusicMaker music;

    public rickroll(MusicMaker maker){
        this.music = maker;


    }
    public void execute(){
        music.LoadSong("rickroll");
        music.Play(true);
    }
    
}
