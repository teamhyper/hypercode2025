package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

public final class Constants {
    public static final PIDConstants translationPidConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants rotationPidConstants = new PIDConstants(5.0, 0.0, 0.0);
    
    public static final double xSpeedLimiter = 3.0;
    public static final double ySpeedLimiter = 3.0;
    public static final double rotLimiter = 3.0;
}
