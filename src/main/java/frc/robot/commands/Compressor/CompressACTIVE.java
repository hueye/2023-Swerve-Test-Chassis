package frc.robot.commands.Compressor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CompressACTIVE extends CommandBase {
    
    public CompressACTIVE() {
        addRequirements(RobotContainer.compressorSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        RobotContainer.compressorSystem.run();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
