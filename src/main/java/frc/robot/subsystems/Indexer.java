package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    SparkMax motor;
    public double velocidad = Constants.Indexer.kVelocidad;



    public Indexer() {
        motor = new SparkMax(Constants.Indexer.kMotorID, SparkMax.MotorType.kBrushless);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kCoast);
        motorConfig.smartCurrentLimit(65);
        motorConfig.inverted(false);
        motorConfig.openLoopRampRate(0.25);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putNumber("Indexer/Velocidad", velocidad);
    }


    public Command encender (){
        return this.runEnd(
            () -> {
                double velocidadLeida = SmartDashboard.getNumber("Indexer/velocidad", velocidad);
                motor.set(velocidadLeida);
            },
            () -> motor.stopMotor() 
            );
    }

    public Command alRevez (){
        return this.runEnd(
            () -> {
            double velocidadLeida = SmartDashboard.getNumber("Indexer/velocidad" ,-velocidad);
                motor.set(velocidadLeida);
            }
            , () -> {
                motor.stopMotor();
            });
    }

    public Command ON(){
        return this.runOnce(
            ()-> motor.set(velocidad)
        );
    }

    public Command OFF(){
        return this.runOnce(
            ()-> motor.set(-0.5)
        );
    }

    public Command movimiento(){
        return Commands.sequence(
            ON(),
            Commands.waitSeconds(2),
            OFF(),
            Commands.waitSeconds(0.1)
            
        
            
        ).repeatedly().finallyDo(
            ()-> motor.set(0)
        );
    }

    public Command pasandopelotas(){
        return Commands.sequence(
            movimiento().withTimeout(5),
            encender()
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer/Corriente", motor.getOutputCurrent());
        SmartDashboard.putNumber("Indexer/Velocidad", motor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Indexer/Posicion", motor.getEncoder().getPosition());
        
    }




}
