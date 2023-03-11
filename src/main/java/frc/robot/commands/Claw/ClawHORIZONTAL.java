package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClawHORIZONTAL extends CommandBase {
    
    public ClawHORIZONTAL() {
        addRequirements(RobotContainer.clawSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        RobotContainer.clawSystem.openClaw();   
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.clawSystem.closeClaw();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
