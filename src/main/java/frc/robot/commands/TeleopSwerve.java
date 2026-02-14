package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
    private final SwerveBase s_Swerve;
    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;
    private final DoubleSupplier rotation;
    private final DoubleSupplier turbo;
    private final BooleanSupplier toggleRobotCentric;
    private BooleanSupplier alignToMoveSup;
    private PIDController headingController = new PIDController(0.022, 0.0, 0.0000);
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Swerve.kSlewRateLimit);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Swerve.kSlewRateLimit);

    private boolean robotCentric = false;
    private boolean lastButtonState = false;
    

    public TeleopSwerve(
            SwerveBase s_Swerve,
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier rotation,
            DoubleSupplier turbo,
            BooleanSupplier toggleRobotCentric,
            BooleanSupplier alignToMoveSup) {

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;
        this.turbo = turbo;
        this.toggleRobotCentric = toggleRobotCentric;
        this.alignToMoveSup = alignToMoveSup;

        headingController.enableContinuousInput(-180, 180);
        SmartDashboard.putData("Swerve/HeadingPID", headingController);
    }

   @Override
    public void execute() {
        // 1. Leer valores (Ya lo tienes)
        double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.OIConstants.kStickDeadband);
        double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.OIConstants.kStickDeadband);
        double rotationVal = MathUtil.applyDeadband(-rotation.getAsDouble(), Constants.OIConstants.kStickDeadband);

        double xFiltered = xLimiter.calculate(translationVal);
        double yFiltered = yLimiter.calculate(strafeVal);

        // --- DEBUGGING: Ver si el botón se detecta ---
        boolean isButtonPushed = alignToMoveSup.getAsBoolean();
        boolean isMoving = (Math.abs(translationVal) > 0.01 || Math.abs(strafeVal) > 0.01);
        
        SmartDashboard.putBoolean("Debug/Boton Presionado", isButtonPushed);
        SmartDashboard.putBoolean("Debug/Se Mueve", isMoving);
        // ---------------------------------------------

        if (isButtonPushed) {
            if (isMoving) {
                // Calcular ángulo
                double targetAngle = Math.toDegrees(Math.atan2(strafeVal, translationVal)); 
                
                // Calcular PID
                double currentAngle = s_Swerve.getYaw().getDegrees();
                double pidOutput = headingController.calculate(currentAngle, targetAngle);
                
                // Sobrescribir rotación
                rotationVal = MathUtil.clamp(pidOutput, -1.0, 1.0);

                // --- DEBUGGING: Ver valores matemáticos ---
                SmartDashboard.putNumber("Debug/Target Angle", targetAngle);
                SmartDashboard.putNumber("Debug/Current Angle", currentAngle);
                SmartDashboard.putNumber("Debug/PID Output", pidOutput);
                // ------------------------------------------
            }
        } else {
             SmartDashboard.putNumber("Debug/PID Output", 0.0); // Reset para no confundirnos
        }

        // Lógica de Turbo y Centric (Tu código original)
        boolean speedCutoffVal = turbo.getAsDouble() <= 0.1;
        boolean currentButtonState = toggleRobotCentric.getAsBoolean();
        if (currentButtonState && !lastButtonState) {
            robotCentric = !robotCentric;
        }
        lastButtonState = currentButtonState;

        s_Swerve.drive(
            new Translation2d(xFiltered, yFiltered)
                    .times(Constants.Swerve.kMaxSpeed)
                    .times(speedCutoffVal ? 1 : 0.5),
            rotationVal * Constants.Swerve.kMaxAngularVelocity * (speedCutoffVal ? 0.5 : 1),
            !robotCentric, 
            Constants.Swerve.kIsOpenLoopTeleopSwerve
        );
    }
}
