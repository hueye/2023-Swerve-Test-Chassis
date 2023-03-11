package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    
    //The arm has two solenoids on either side. 
    
    public static DoubleSolenoid leftArmSolenoid;

    public static DoubleSolenoid rightArmSolenoid;

    public Arm() {
        leftArmSolenoid = new DoubleSolenoid(
            Constants.COMPRESSOR_ID,
            PneumaticsModuleType.REVPH,
            Constants.ARM_EXTEND_CHANNEL,
            Constants.ARM_RETRACT_CHANNEL
        );

        rightArmSolenoid = new DoubleSolenoid(
            Constants.COMPRESSOR_ID,
            PneumaticsModuleType.REVPH,
            Constants.ARM_EXTEND_CHANNEL,
            Constants.ARM_RETRACT_CHANNEL
        );
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    public void Extended() {
        leftArmSolenoid.set(Value.kForward);
        rightArmSolenoid.set(Value.kForward);
    }

    public void Retracted() {
        leftArmSolenoid.set(Value.kReverse);
        rightArmSolenoid.set(Value.kReverse);
    }
}
