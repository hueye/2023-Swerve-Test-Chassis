package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//The claw has two solenoids - one operating the claw to close and open it, and one to
//move it up and down. The values may need to be adjusted, as they do not have finite
//values to move in and out or up and down.

public class Claw extends SubsystemBase {
    
    public DoubleSolenoid miniClawSolenoid;

    public DoubleSolenoid bigClawSolenoid;

    public Claw(){

        miniClawSolenoid = new DoubleSolenoid(
            Constants.COMPRESSOR_ID,
            PneumaticsModuleType.REVPH,
            Constants.CLAW_OUT_CHANNEL,
            Constants.CLAW_IN_CHANNEL);

        bigClawSolenoid = new DoubleSolenoid(
            Constants.COMPRESSOR_ID,
            PneumaticsModuleType.REVPH,
            Constants.CLAW_EXTEND_SOLENOID_CHANNEL,
            Constants.CLAW_RETRACT_SOLENOID_CHANNEL);
    }
    
    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    public void openClaw() {
        miniClawSolenoid.set(Value.kForward);
    }

    public void closeClaw() {
        miniClawSolenoid.set(Value.kReverse);
    }

    
    public void moveClawUp() {
        bigClawSolenoid.set(Value.kForward);
    }

    public void moveClawDown() {
        bigClawSolenoid.set(Value.kReverse);
    }
}

