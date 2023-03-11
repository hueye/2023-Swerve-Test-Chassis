package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

//Declaring swerve modules on the robot
//fL = front left, fR = front right, bL = back left, bR = back right
    final SwerveModule fL = new SwerveModule(
        Constants.kFrontLeftDrivingCanId,
        Constants.kFrontLeftTurningCanId,
        Constants.kFrontLeftChassisAngularOffset);

    final SwerveModule fR = new SwerveModule(
        Constants.kFrontRightDrivingCanId,
        Constants.kFrontRightTurningCanId,
        Constants.kFrontRightChassisAngularOffset);

    final SwerveModule bL = new SwerveModule(
        Constants.kRearLeftDrivingCanId,
        Constants.kRearLeftTurningCanId,
        Constants.kBackLeftChassisAngularOffset);

    final SwerveModule bR = new SwerveModule(
        Constants.kRearRightDrivingCanId,
        Constants.kRearRightTurningCanId,
        Constants.kBackRightChassisAngularOffset);

//Gyro for the drivetrain
    final WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.pigeonid);

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        Constants.kDriveKinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            fL.getPosition(),
            fR.getPosition(),
            bL.getPosition(),
            bR.getPosition(),
        });
       
//Finding position of the robot
    @Override
    public void periodic() {
      odometry.update(
      gyro.getRotation2d(),
          new SwerveModulePosition[] {
              fL.getPosition(),
              fR.getPosition(),
              bL.getPosition(),
              bR.getPosition()
  });
  
  SmartDashboard.putNumber("gyro", gyro.getRotation2d().getDegrees());
}
 
  public Pose2d getPose() {
        return odometry.getPoseMeters();
      }

  public void resetOdometry(Pose2d pose) {
     odometry.resetPosition(
       gyro.getRotation2d(),

         new SwerveModulePosition[] {
            fL.getPosition(),
            fR.getPosition(),
            bL.getPosition(),
            bR.getPosition()
              },
            pose);
        }

//Drive subsystem - called in the swerve command
        public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
          xSpeed *= Constants.kMaxSpeedMetersPerSecond;
          ySpeed *= Constants.kMaxSpeedMetersPerSecond;
          rot *= Constants.kMaxAngularSpeed;

          var swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            
            swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
                fL.setDesiredState(swerveModuleStates[0]);
                fR.setDesiredState(swerveModuleStates[1]);
                bL.setDesiredState(swerveModuleStates[2]);
                bR.setDesiredState(swerveModuleStates[3]);

    if(fieldRelative)
        {
          SmartDashboard.putString("Orientation", "Field Oriented");
        } else {
          SmartDashboard.putString("Orientation", "Robot Oriented");
        }
      }
      
        public void setX() {
          fL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          fR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          bL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          bR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        }

        public void setModuleStates(SwerveModuleState[] desiredStates) {
          SwerveDriveKinematics.desaturateWheelSpeeds(
              desiredStates, Constants.kMaxSpeedMetersPerSecond);
          fL.setDesiredState(desiredStates[0]);
          fR.setDesiredState(desiredStates[1]);
          bL.setDesiredState(desiredStates[2]);
          bR.setDesiredState(desiredStates[3]);
        }
      
        public void resetEncoders() {
          fL.resetEncoders();
          fR.resetEncoders();
          bL.resetEncoders();
          bR.resetEncoders();
        }
      
        public void zeroHeading() {
          gyro.reset();
        }

        public double getHeading() {
          return gyro.getRotation2d().getDegrees();
        }
     
        public double getTurnRate() {
          return gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
        }

      
      }
      