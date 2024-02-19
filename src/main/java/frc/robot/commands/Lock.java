package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class Lock extends CommandBase{
    private final Swerve swerve;

    public Lock(Swerve swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        swerve.setLock(!swerve.getLockedState());
    }

    @Override
    public void end(boolean interrupted){
        swerve.stopModules();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}