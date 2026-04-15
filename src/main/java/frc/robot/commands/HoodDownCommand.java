package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.turret.HoodSubsystem;


public class HoodDownCommand extends Command {

    private final HoodSubsystem hood;
    
    public HoodDownCommand(
        HoodSubsystem hood
    ) {
        
        this.hood = hood;
        
        addRequirements( hood );
    }

    @Override
    public void execute() {
        hood.moveDownServo();
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
