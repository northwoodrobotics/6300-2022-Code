package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SetServoZero extends CommandBase{
    private ShooterSubsystem shooter;

    public SetServoZero(ShooterSubsystem subsystem){
        this.shooter = subsystem;
        addRequirements(subsystem);
    }
    public void execute(){
        shooter.MoveHood(-30);
    }
}