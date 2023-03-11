package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    
    CANSparkMax driveMotor;
    CANSparkMax turnMotor;
  
    PIDController drivePID = new PIDController(Constants.kDrivingP, Constants.kDrivingI, Constants.kDrivingD);
    PIDController turnPID = new PIDController(Constants.kTurningP, Constants.kTurningI, Constants.kTurningD);
  
    CANCoder driveEncoder;
    CANCoder turnEncoder;
  
    double chassisAngularOffset;
  
    SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int drivingCANID, int turningCANID, double chassisAngularOffset) {
        driveMotor = new CANSparkMax(drivingCANID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(drivingCANID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.set(drivePID.calculate(driveEncoder.getPosition()));
        turnMotor.set(turnPID.calculate(driveEncoder.getPosition()));

        driveEncoder.setPosition(Constants.kDrivingEncoderPositionFactor);
        driveEncoder.configMagnetOffset(Constants.kDrivingEncoderVelocityFactor);

        turnEncoder.setPosition(Constants.kTurningEncoderPositionFactor);
        turnEncoder.configMagnetOffset(Constants.kTurningEncoderVelocityFactor);
        turnEncoder.configSensorDirection(true);

        turnPID.enableContinuousInput(Constants.kTurningEncoderPositionPIDMinInput, Constants.kTurningEncoderPositionPIDMaxInput);

        driveMotor.setIdleMode(Constants.kDrivingMotorIdleMode);
        driveMotor.setIdleMode(Constants.kTurningMotorIdleMode);
        driveMotor.setSmartCurrentLimit(Constants.kDrivingMotorCurrentLimit);
        driveMotor.setSmartCurrentLimit(Constants.kTurningMotorCurrentLimit);
            
        driveMotor.burnFlash();
        turnMotor.burnFlash();
        
        desiredState.angle = new Rotation2d(turnEncoder.getPosition());
        driveEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(),
            new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(),
            new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
      }

      public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
    
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(turnEncoder.getPosition()));
    
        drivePID.setSetpoint(optimizedDesiredState.speedMetersPerSecond);
        turnPID.setSetpoint(optimizedDesiredState.angle.getRadians());
    
      }
    
      /** Zeroes all the SwerveModule encoders. */
      public void resetEncoders() {
        driveEncoder.setPosition(0);
      }
}
