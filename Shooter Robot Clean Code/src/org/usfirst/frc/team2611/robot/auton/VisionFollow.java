package org.usfirst.frc.team2611.robot.auton;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionFollow {

    public static PID turn, distance;
    public static boolean run;
    public static double right_motor_turn_value, left_motor_turn_value, right_motor_dis_value, left_motor_dis_value;
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");



    public static void Follow(){

        NetworkTableEntry ta = table.getEntry("ta");

        NetworkTableEntry tx = table.getEntry("tx");

        double xCords   = tx.getDouble(0);
        double area     = ta.getDouble(0);

        run = true;

        turn        = new PID(.005, 0 , 0);
        distance    = new PID(.005,0,0);

        while (run) {

            right_motor_turn_value  = turn.PIDInput(xCords, 0);
            left_motor_turn_value   = right_motor_turn_value * -1;

            right_motor_dis_value   = distance.PIDInput(area, 2000);
            left_motor_dis_value    = right_motor_dis_value;

        }
    }
}
