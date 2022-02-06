package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class MusicMaker extends SubsystemBase{
    private TalonFX instrument1 = new TalonFX(DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR);
    
    private TalonFX instrument2 = new TalonFX(DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR);

    private TalonFX instrument3 = new TalonFX(DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
    
    private TalonFX instrument4 = new TalonFX(DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR);

    private TalonFX instrument5 = new TalonFX(DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR);
    
    private TalonFX instrument6 = new TalonFX(DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR);

    private TalonFX instrument7 = new TalonFX(DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR);
    
    private TalonFX instrument8 = new TalonFX(DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR);


    private Orchestra DrivetrainOrchestra;
    public String Song;


    public MusicMaker(){
        DrivetrainOrchestra.addInstrument(instrument1);
        DrivetrainOrchestra.addInstrument(instrument2);
        DrivetrainOrchestra.addInstrument(instrument3);
        DrivetrainOrchestra.addInstrument(instrument4);
        DrivetrainOrchestra.addInstrument(instrument5);
        DrivetrainOrchestra.addInstrument(instrument6);
        DrivetrainOrchestra.addInstrument(instrument7);
        DrivetrainOrchestra.addInstrument(instrument8);





    }

    public void LoadSong(String song){
        DrivetrainOrchestra.loadMusic(song);
        song = Song;
    }

    public String CurrentSong(){
        return Song;
    }

    public void Play(boolean play){
        DrivetrainOrchestra.isPlaying();
    }
    

    





    



    
}
