package frc.robot.subsystems.Vision;

import java.util.concurrent.locks.Condition;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;
import frc.robot.team696.LimeLightCam;
import frc.robot.RobotMap.PoseConfig;
import frc.robot.Constants;

/** Reports our expected, desired, and actual poses to dashboards */
public class PoseEstimator extends SubsystemBase {

  // PoseConfig config;
  public Field2d field = Constants.Swerve.field;

  private Pose2d estimatePose = new Pose2d();

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Swerve drivetrain;

  LimeLightCam cam = new LimeLightCam("limelight-a");

  public PoseEstimator(Swerve swerve) {

    drivetrain = swerve;

    // Maxswerve Version from MAXSwerve.java in core
    poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Swerve.swerveKinematics,
        drivetrain.getGyroYaw(),
        drivetrain.getModulePositions(),
        new Pose2d(),
        createStateStdDevs(
            PoseConfig.kPositionStdDevX,
            PoseConfig.kPositionStdDevY,
            PoseConfig.kPositionStdDevTheta),
        createVisionMeasurementStdDevs(
            PoseConfig.kVisionStdDevX,
            PoseConfig.kVisionStdDevY,
            PoseConfig.kVisionStdDevTheta));

  }

  @Override
  public void periodic() {
    Constants.AutoConstants.setLimelightStatus(SmartDashboard.getBoolean("Limelight Status", false));
    updateOdometryEstimate();

    cam.updateEstimator(drivetrain.getGyroYaw().getDegrees(), poseEstimator, () -> {
      return true;
    });

    field.setRobotPose(getPose());
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> field.getObject(Constants.AutoConstants.getAutoSelector().getSelected()).setPoses(poses));

    SmartDashboard.putData(field);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometryEstimate() {
    poseEstimator.update(drivetrain.getGyroYaw(), drivetrain.getModulePositions());
  }

  /**
   * @see edu.wpi.first.math.estimator.PoseEstimator#addVisionMeasurement(Pose2d,
   *      double)
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d newPose) {
    poseEstimator.resetPosition(drivetrain.getGyroYaw(), drivetrain.getModulePositions(), newPose);
  }

  /**
   * Reset the pose estimator location and Drivetrain odometry - NEEDS TO BE
   * TESTED
   *
   * @param poseMeters
   */
  public void resetPoseEstimate(Pose2d poseMeters) {
    poseEstimator.resetPosition(drivetrain.getGyroYaw(), drivetrain.getModulePositions(), poseMeters);
  }

  public void resetHeading(Rotation2d angle) {
    resetPoseEstimate(new Pose2d(estimatePose.getTranslation(), angle));
  }

  public void resetLocationEstimate(Translation2d translation) {
    resetPoseEstimate(new Pose2d(translation, new Rotation2d(0)));
  }

  /**
   * Gets the pose of the robot at the current time as estimated by the
   * poseEstimator. This includes
   * vision and odometry combined together.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Get the heading of the robot estimated by the poseEstimator. Use this in most
   * places we would
   * use the gyro.
   *
   * @return
   */
  public Rotation2d getHeading() {
    return estimatePose.getRotation();
  }

  public Translation2d getLocation() {
    return estimatePose.getTranslation();
  }

  public Pose2d getEstimatedPose() {
    return estimatePose;
  }

  /**
   * Creates a vector of standard deviations for the states. Standard deviations
   * of model states.
   * Increase these numbers to trust your model's state estimates less.
   *
   * @param x     in meters
   * @param y     in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createStateStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  /**
   * Creates a vector of standard deviations for the local measurements. Standard
   * deviations of
   * encoder and gyro rate measurements. Increase these numbers to trust sensor
   * readings from
   * encoders and gyros less.
   *
   * @param theta in degrees per second
   * @param s     std for all module positions in meters per sec
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N5> createLocalMeasurementStdDevs(double theta, double s) {
    return VecBuilder.fill(Units.degreesToRadians(theta), s, s, s, s);
  }

  /**
   * Creates a vector of standard deviations for the vision measurements. Standard
   * deviations of
   * global measurements from vision. Increase these numbers to trust global
   * measurements from
   * vision less.
   *
   * @param x     in meters
   * @param y     in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  /**
   * Commnad to reset odometry of drivetrain and pose esimator to the one from
   * vision
   * 
   * @return a command to reset the Pose Estimator and Drivetrain to the vision
   *         pose
   */

}
