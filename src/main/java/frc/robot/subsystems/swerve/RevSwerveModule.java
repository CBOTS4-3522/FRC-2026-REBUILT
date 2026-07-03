/*
 * RevSwerveModule.java
 * 
 * Implementación concreta de un módulo Swerve utilizando hardware de REV Robotics 
 * (Controladores SPARK MAX, motores NEO) y un encoder absoluto CANCoder de CTRE.
 * Integra capacidades completas de simulación física (DCMotorSim).
 */
package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.swerveUtil.CTREState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class RevSwerveModule implements SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    
    private SparkMax mAngleMotor;
    private SparkMax mDriveMotor;
    
    // Modelos matemáticos de estado en el espacio (State-Space) para simulación
    private DCMotorSim mDriveSim;
    private DCMotorSim mAngleSim;
    
    // Objetos de configuración de la nueva API 2025 de REV
    private SparkMaxConfig mAngleConfig;
    private SparkMaxConfig mDriveConfig;
    
    private CANcoder angleEncoder;
    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;
    
    public SwerveModuleState desiredState;
    public final FeedForwardConfig feedForward = new FeedForwardConfig();
    private double lastVelocitySetpoint = 0.0;

    public RevSwerveModule(int moduleNumber, RevSwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Inicialización del hardware físico */
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAngleConfig = new SparkMaxConfig();
        configAngleMotor(); 

        mDriveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        mDriveConfig = new SparkMaxConfig();
        configDriveMotor(); 

        angleEncoder = new CANcoder(moduleConstants.cancoderID);

        // Se aplican las configuraciones preparadas a los controladores
        configEncoders();
        
        // Alineación inicial crítica: Encoder Relativo <- Encoder Absoluto
        synchronizeEncoders();
        
        this.desiredState = new SwerveModuleState(0, getAngle());

        // -------------------------------------------------------------------
        // MODELADO MATEMÁTICO PARA SIMULACIÓN
        // -------------------------------------------------------------------
        // Creamos "plantas" (modelos físicos) estimando la inercia del sistema 
        // y la relación de reducción mecánica.
        var drivePlant = LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), 0.025, Constants.Swerve.kDriveGearRatio
        );
        var anglePlant = LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), 0.004, Constants.Swerve.kAngleGearRatio
        );

        // Instanciamos los simuladores de los motores
        mDriveSim = new DCMotorSim(drivePlant, DCMotor.getNEO(1));
        mAngleSim = new DCMotorSim(anglePlant, DCMotor.getNEO(1));
    }

    /** Configura los factores de conversión para que los SparkMax trabajen en Metros y Grados nativamente. */
    private void configEncoders() {
        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);
        mDriveConfig.encoder.positionConversionFactor(Constants.Swerve.kDriveRevToMeters);
        mDriveConfig.encoder.velocityConversionFactor(Constants.Swerve.kDriveRpmToMetersPerSecond);

        relAngleEncoder = mAngleMotor.getEncoder();
        mAngleConfig.encoder.positionConversionFactor(Constants.Swerve.kDegreesPerTurnRotation);
        mAngleConfig.encoder.velocityConversionFactor(Constants.Swerve.kDegreesPerTurnRotation / 60);

        mDriveMotor.configure(mDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        mAngleMotor.configure(mAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** Configura las ganancias PID y límites de corriente del motor de dirección. */
    private void configAngleMotor() {
        mAngleConfig.closedLoop.p(Constants.Swerve.Angle.kP, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.i(Constants.Swerve.Angle.kI, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.d(Constants.Swerve.Angle.kD, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.feedForward.kS(Constants.Swerve.Angle.kS, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.feedForward.kV(Constants.Swerve.Angle.kV, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.feedForward.kA(Constants.Swerve.Angle.kA, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.outputRange(-Constants.Swerve.kAnglePower, Constants.Swerve.kAnglePower);
        
        mAngleConfig.smartCurrentLimit(Constants.Swerve.kAngleContinuousCurrentLimit);
        mAngleConfig.inverted(Constants.Swerve.kAngleMotorInvert);
        mAngleConfig.idleMode(Constants.Swerve.kAngleIdleMode);
        mAngleConfig.closedLoopRampRate(Constants.Swerve.kAngleRampRate);
    }

    /** Configura las ganancias PID y límites de corriente del motor de tracción. */
    private void configDriveMotor() {
        mDriveConfig.voltageCompensation(12.0); // Previene comportamientos extraños si la batería baja
        mDriveConfig.closedLoop.p(Constants.Swerve.Drive.kP, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.i(Constants.Swerve.Drive.kI, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.d(Constants.Swerve.Drive.kD, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.feedForward.kA(Constants.Swerve.Drive.kA, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.feedForward.kV(Constants.Swerve.Drive.kV, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.feedForward.kS(Constants.Swerve.Drive.kS, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.outputRange(-12.0, 12.0);
        
        mDriveConfig.smartCurrentLimit(Constants.Swerve.kDriveContinuousCurrentLimit);
        mDriveConfig.inverted(Constants.Swerve.kDriveMotorInvert);
        mDriveConfig.idleMode(Constants.Swerve.kDriveIdleMode);
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean forceAngle) {
        // Optimización del estado: Evita que el módulo gire más de 90 grados; invierte la velocidad si es necesario.
        this.desiredState = CTREState.optimize(desiredState, getState().angle);
        setAngle(this.desiredState, forceAngle);
        setSpeed(this.desiredState, isOpenLoop);
    }
    
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Anti-Jitter: Si el joystick se suelta, apagar motor.
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            mDriveMotor.stopMotor();
            return;
        }

        if (isOpenLoop) {
            // Control por porcentaje (Teleoperado simple)
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.kMaxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }

        // Control Lazo Cerrado (Autónomo o perfiles de trayectoria)
        double velocity = desiredState.speedMetersPerSecond;
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        controller.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    private void setAngle(SwerveModuleState desiredState, boolean forceAngle) {
        // Anti-Jitter: No desgastar los engranes corrigiendo el ángulo si el robot no se está moviendo.
        if (!forceAngle && Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.kMaxSpeed * 0.01)) {
            mAngleMotor.stopMotor();
            return;
        }
             
        Rotation2d angle = desiredState.angle;
        SparkClosedLoopController controller = mAngleMotor.getClosedLoopController();
        double degReference = angle.getDegrees();
        controller.setSetpoint(degReference, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    @Override
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    @Override
    public int getModuleNumber() {
        return moduleNumber;
    }

    @Override
    public void setModuleNumber(int moduleNumber) {
        this.moduleNumber = moduleNumber;
    }

    @Override
    public void synchronizeEncoders() {
        // Resta el imán físico del módulo (offset) para alinear las ruedas hacia el frente.
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        relAngleEncoder.setPosition(absolutePosition);
    }

    @Override
    public SwerveModuleState getState() {
        // Abstracción transparente: La capa superior (SwerveBase) no necesita saber si es simulación o hardware.
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            double simVel = mDriveSim.getAngularVelocityRadPerSec() * (Constants.Swerve.kWheelCircumference / (2 * Math.PI));
            return new SwerveModuleState(simVel, getAngle());
        }
        return new SwerveModuleState(relDriveEncoder.getVelocity(), getAngle());
    }

    @Override
    public double getOmega() {
        return angleEncoder.getVelocity().getValueAsDouble() * 360;                       
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                relDriveEncoder.getPosition(),
                getAngle());
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public void setDriveVoltage(double volts) {
        mDriveMotor.setVoltage(volts);
    }

    @Override
    public double getDriveVoltage() {
        return mDriveMotor.getBusVoltage() * mDriveMotor.getAppliedOutput();
    }

    @Override
    public void lockAngle() {
        mAngleMotor.getClosedLoopController().setSetpoint(0, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }

    public void updateDrivePID(double p, double d, double i) {
        mDriveConfig.closedLoop.p(p, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.d(d, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.i(i, ClosedLoopSlot.kSlot0);
        
        mDriveMotor.configure(mDriveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Bucle de simulación ejecutado periódicamente (20ms).
     * Modela la respuesta de los motores basados en la cinemática deseada inyectando voltaje 
     * a las plantas virtuales y actualizando los valores de los encoders simulados.
     */
    public void simulationPeriodic(double dt) {
        // 1. LÓGICA DE CONTROL (Simulación del Firmware del SparkMax)
        double velocitySetpoint = desiredState.speedMetersPerSecond;
        double currentSimVelocity = mDriveSim.getAngularVelocityRadPerSec() * (Constants.Swerve.kWheelCircumference / (2 * Math.PI));
        
        double accelerationSetpoint = (velocitySetpoint - lastVelocitySetpoint) / dt;
        lastVelocitySetpoint = velocitySetpoint;

        // FeedForward Híbrido (Ecuación Física de Sistema)
        double driveVoltage = 0.0;
        if (Math.abs(velocitySetpoint) > 0.01) {
            driveVoltage += Math.signum(velocitySetpoint) * Constants.Swerve.Drive.kS; // Fricción
        }
        driveVoltage += velocitySetpoint * Constants.Swerve.Drive.kV; // Vel.
        driveVoltage += accelerationSetpoint * Constants.Swerve.Drive.kA; // Acel.

        // Cálculo del Lazo Cerrado (Proporcional)
        double velocityError = velocitySetpoint - currentSimVelocity;
        driveVoltage += velocityError * (Constants.Swerve.Drive.kP * 12.0); 

        double angleErrorDegrees = desiredState.angle.minus(getAngle()).getDegrees();
        double angleVoltage = angleErrorDegrees * (Constants.Swerve.Angle.kP * 12.0);

        driveVoltage = MathUtil.clamp(driveVoltage, -12.0, 12.0);
        angleVoltage = MathUtil.clamp(angleVoltage, -12.0, 12.0);

        // 2. APLICACIÓN FÍSICA AL MUNDO REAL SIMULADO
        mDriveSim.setInputVoltage(driveVoltage);
        mAngleSim.setInputVoltage(angleVoltage);
        mDriveSim.update(dt);
        mAngleSim.update(dt);

        // 3. ACTUALIZACIÓN DE SENSORES RELATIVOS
        double drivePosMeters = (mDriveSim.getAngularPositionRad() / (2 * Math.PI)) * Constants.Swerve.kWheelCircumference;
        relDriveEncoder.setPosition(drivePosMeters);
                 
        double angleDegrees = Math.toDegrees(mAngleSim.getAngularPositionRad());
        relAngleEncoder.setPosition(angleDegrees);
    }
}