package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.turret.HoodSubsystem;


public class HoodUpCommand extends Command {

    private final HoodSubsystem hood;
    
    public HoodUpCommand(
        HoodSubsystem hood
    ) {
        
        this.hood = hood;
        
        addRequirements( hood );
    }

    @Override
    public void execute() {
        hood.moveUpServo();
       // System.out.println("Hood Up");
    }

    @Override
    public void end(boolean interrupted) {
        
        hood.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
