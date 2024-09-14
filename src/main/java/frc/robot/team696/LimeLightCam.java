package frc.robot.team696;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import frc.robot.subsystems.Vision.LimelightHelpers;


public class LimeLightCam extends BaseCam {
    public String name = "";
    public static int LimeLightCount = 0;
    
    public LimeLightCam(String name, int[] TagsToCheck) {
        this.name = name;

        if(TagsToCheck.length > 0) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, TagsToCheck); 
        }

        for (int port = 5800; port <= 5809; port++) { 
            PortForwarder.add(port + 10 * LimeLightCount, String.format("%s.local", this.name), port);
        }

        LimeLightCount++;
    }

    public LimeLightCam(String name) {
        this(name, new int[] {});
    }

    boolean hasTarget() {
        return LimelightHelpers.getTargetCount(name) > 0;
    }

    double tX() {
        return LimelightHelpers.getTX(name);
    }

    public Optional<AprilTagResult> getEstimate(double curYaw) {
        LimelightHelpers.SetRobotOrientation(name, curYaw,0,0,0,0,0);
        LimelightHelpers.PoseEstimate latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (latestEstimate == null) return Optional.empty();

        if (latestEstimate.pose.equals(new Pose2d())) return Optional.empty();

        return Optional.of(
            new AprilTagResult(latestEstimate.pose, 
                latestEstimate.timestampSeconds, 
                0, 
                0,
                0)); // Probably not the best but good enough for now
    }

}
