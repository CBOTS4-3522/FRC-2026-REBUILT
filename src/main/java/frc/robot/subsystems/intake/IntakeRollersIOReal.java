package frc.robot.subsystems.intake;

import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class IntakeRollersIOReal implements IntakeRollersIO {

    private final SparkMax motorRodillos;
   

    public IntakeRollersIOReal() {
        motorRodillos = new SparkMax(Constants.Intake.kMotorRollerID, MotorType.kBrushless);


        SparkMaxConfig configRodillos = new SparkMaxConfig();
        configRodillos.idleMode(IdleMode.kCoast);
        configRodillos.smartCurrentLimit(70);

        

        motorRodillos.configure(configRodillos, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    @Override
    public void updateInputs(IntakeRollersIOInputs inputs) {
        inputs.rodillosCorriente = motorRodillos.getOutputCurrent();
        inputs.rodillosVelocidad = motorRodillos.getEncoder().getVelocity();
        inputs.rodillosVoltajeAplicado = motorRodillos.getBusVoltage() * motorRodillos.getAppliedOutput();

        
    }

   

    @Override
    public void setVoltajeRodillos(double volts) {
        motorRodillos.setVoltage(volts);
    }

    @Override
    public void stopRodillos() {
        motorRodillos.stopMotor();
    }

  
}