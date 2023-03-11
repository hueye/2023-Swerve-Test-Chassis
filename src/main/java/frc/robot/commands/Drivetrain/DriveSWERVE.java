package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class DriveSWERVE extends CommandBase {
    CommandXboxController driveCtrl;
    boolean fieldOriented = false;

    public DriveSWERVE(CommandXboxController driverController, boolean fieldOriented) {
        addRequirements(RobotContainer.drivetrain);

        this.driveCtrl = driverController;
        this.fieldOriented = fieldOriented;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        RobotContainer.drivetrain.drive(
            MathUtil.applyDeadband(driveCtrl.getLeftX(), 0.05),
            MathUtil.applyDeadband(driveCtrl.getLeftY(), 0.05),
            MathUtil.applyDeadband(driveCtrl.getRightX(), 0.05), fieldOriented);}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
