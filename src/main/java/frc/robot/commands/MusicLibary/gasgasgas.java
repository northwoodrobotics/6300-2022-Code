package frc.robot.commands.MusicLibary;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MusicMaker;

public class gasgasgas extends CommandBase{
    private MusicMaker music;

    public gasgasgas(MusicMaker maker){
        this.music = maker;


    }
    public void execute(){
        music.LoadSong("gasgasgas");
        music.Play(true);
    }
    
}
