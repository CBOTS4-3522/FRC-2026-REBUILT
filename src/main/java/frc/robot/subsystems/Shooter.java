package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.shooter;

public class Shooter implements Subsystem {

    public SparkMax pivotMotor = new SparkMax(Constants.shooter.pivot.kID, MotorType.kBrushless);

    public SparkMaxConfig pivotConfig = new SparkMaxConfig();


    public Shooter() {

        pivotConfig.idleMode(IdleMode.kCoast);
        pivotConfig.smartCurrentLimit(40);
        pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Shooter/Pivot/Percent",0.2);

    }

    //metodo que le pone un porcentaje al motor
    public void setPivotPercent(double percent) {
        pivotMotor.set(percent);
    }

    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    public Command girarShooter(){
        return this.runEnd(
            ()-> setPivotPercent(SmartDashboard.getNumber("Shooter/Pivot/Percent", 0.2)),
            ()-> stopPivot()
            );
    }

    public Command girarShooterAlrevez(){
        return this.runEnd(
            ()-> setPivotPercent(-SmartDashboard.getNumber("Shooter/Pivot/Percent", 0.2)),
            ()-> stopPivot()
            );
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Pivot/Current", pivotMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Pivot/Velocity", pivotMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Pivot/Position", pivotMotor.getEncoder().getPosition());
    }


}
