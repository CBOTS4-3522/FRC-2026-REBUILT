package frc.lib.util.swerveUtil;

import edu.wpi.first.math.util.Units;

    public class SwerveConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final double angleKF;
    public final boolean driveMotorInvert;
    public final boolean angleMotorInvert;
    public final boolean canCoderInvert;

    public SwerveConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, 
    double angleKP,double angleKI, double angleKD, double angleKF, 
    boolean driveMotorInvert, boolean angleMotorInvert, boolean canCoderInvert){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.angleKF = angleKF;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.canCoderInvert = canCoderInvert;
    }

    /** Swerve Drive Specialties - MK4i Module*/
    public static SwerveConstants SDSMK4i(double driveGearRatio){

//Diametro
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (150.0 / 7.0);

        double angleKP = 0.3;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = true;
        boolean canCoderInvert = false;
        return new SwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, 
        angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, canCoderInvert);
    }

    /* Drive Gear Ratios for all supported modules */
    public class driveGearRatios{
        /** SDS MK4i - 6.75 : 1 */
        public static final double SDSMK4i_L2 = (6.75);
    }
}