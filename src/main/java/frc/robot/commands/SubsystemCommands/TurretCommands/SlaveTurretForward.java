package frc.robot.commands.SubsystemCommands.TurretCommands;

import org.intellij.lang.annotations.JdkConstants.BoxLayoutAxis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class SlaveTurretForward extends CommandBase{
    private final TurretSubsystem m_turret;
    public SlaveTurretForward(TurretSubsystem turret){
        this.m_turret = turret;
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        m_turret.setTurretAngle(0);
    }

}
