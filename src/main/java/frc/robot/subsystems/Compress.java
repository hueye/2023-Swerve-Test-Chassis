package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Compress extends SubsystemBase {
    
    public Compressor compressor;

    public Compress() {
        compressor = new Compressor(Constants.COMPRESSOR_ID, PneumaticsModuleType.REVPH);
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    public void run() {
        SmartDashboard.putNumber("Pressure Switch Value", compressor.getPressure());
        compressor.enableAnalog(60,120);
    }
}
