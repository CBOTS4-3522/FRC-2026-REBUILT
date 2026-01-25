package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs {
        public boolean hasTarget = false;
        public Pose2d estimatedPose = new Pose2d();
        public double timestamp = 0.0;
        public int[] tagIds = new int[0];
        public double latency = 0.0;
        public double averageDist = 0.0;
    }

    /** Actualiza las entradas desde el hardware */
    public default void updateInputs(VisionIOInputs inputs) {}
}
