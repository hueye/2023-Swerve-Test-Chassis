package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    //Unique variables:
    //tv = whether the limelight has valid targets
    //tx = horizontal offset from crosshair to target
    //ty = vertical offset from crosshair to target
    //ta = target area

    public static NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = limeTable.getEntry("tx");
    NetworkTableEntry ty = limeTable.getEntry("ty");
    NetworkTableEntry ta = limeTable.getEntry("ta");
    public double x, y, area;
    public double[] dis;
    public double distance;
    public double xdistance;

    //LEDMode values for the limelight:
    //1 = force off
    //2 = force blink
    //3 = force on

@Override
public void periodic() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    dis = NetworkTableInstance.getDefault().getTable("limelight").getEntry("bot pose").getDoubleArray(new double[6]);
    distance = dis[1];
    xdistance = dis[0];

    SmartDashboard.putNumberArray("Distance Array", dis);
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("Limelight X", x);
    SmartDashboard.putNumber("Limelight Y", y);
}

public double getVisionX() {
    return x;
}

public double getVisionY() {
    return y;
}

public double getDistance() {
    return distance;
}

public double getDistanceX() {
    return xdistance;
}

}
