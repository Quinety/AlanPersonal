package org.usfirst.frc.team2611.robot.auton;

public class PID {
    
    public static double output;
    
    public double kP, kI, kD;
        
    public static double pError, iError, dError;
    
    public static double loopTime, loopTimePrev, loopTimeDelta;

    public PID(double pGain, double iGain, double dGain){

        kP = pGain;
        kI = iGain;
        kD = dGain;
        loopTime = System.nanoTime();

    }
    
    public double PIDInput(double input, double setpoint){

        loopTimePrev = loopTime;
        loopTime = System.nanoTime();
        loopTimeDelta = loopTime - loopTimePrev;
        System.out.println(loopTime);

        dError = (setpoint - input - (pError)) / loopTimeDelta;
        pError = (setpoint - input);
        iError = (((setpoint - input ) / loopTimeDelta) + iError);

        output = ((pError * kP) + (iError * kI) + (dError * kD));

        
        return output;
    }
    
}
