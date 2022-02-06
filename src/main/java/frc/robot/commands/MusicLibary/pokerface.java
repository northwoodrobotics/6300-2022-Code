package frc.robot.commands.MusicLibary;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MusicMaker;

public class pokerface extends CommandBase{
    private MusicMaker music;

    public pokerface(MusicMaker maker){
        this.music = maker;


    }
    public void execute(){
        music.LoadSong("pokerface");
        music.Play(true);
    }
    
    
    
}
