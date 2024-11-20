package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.Core.FieldData;
import frc.Core.Time;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Devices.Imu;
import frc.Devices.LimeLight;
import frc.lib.math.AngleMath;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Imu gyro;
    public Pose2d startPos;

    public void setStartingPos(Pose2d startPos) {
        this.startPos = startPos;
    }

    public Swerve() {
        startPos = new Pose2d();
        PathPlannerLogging
                .setLogActivePathCallback(
                        (poses) -> Constants.Swerve.field.getObject("path").setPoses(poses));
        gyro = new Imu(Constants.Swerve.pigeonID);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        FieldData.getIsRed() ? getGyroYaw() : new Rotation2d(getGyroYaw().getRadians() + 2 * Math.PI))
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void setBrakeMode() {
        mSwerveMods[0].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        mSwerveMods[1].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        mSwerveMods[2].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        mSwerveMods[3].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public void setCoastMode() {
        mSwerveMods[0].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        mSwerveMods[1].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        mSwerveMods[2].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        mSwerveMods[3].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    }

    public void fromChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public void setGyroYaw(Rotation2d yaw) {
        gyro.setYaw(yaw.getDegrees());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getRobotVelocity() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(this.getModuleStates());
    }

    public void zeroHeading() {
        gyro.resetYaw();
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.updateValues();
        if (DriverStation.isDisabled() && PathPlannerAuto
                .getStaringPoseFromAutoFile(Constants.AutoConstants.getAutoSelector().getSelected()) != startPos) {
            startPos = PathPlannerAuto
                    .getStaringPoseFromAutoFile(Constants.AutoConstants.getAutoSelector().getSelected());
            Constants.Swerve.field.setRobotPose(startPos);
        }
        SmartDashboard.putData(Constants.Swerve.field);
        // poseEstimator.update(getGyroYaw(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Robot Velocity",
                Math.hypot(getRobotVelocity().vxMetersPerSecond, getRobotVelocity().vyMetersPerSecond));

    }
    // if (FieldData.getIsTeleop()) {
    // if (limeLight.botPoseChanged()) {
    // if (limeLight.getPose().getTranslation()
    // .getDistance(poseEstimator.getEstimatedPosition().getTranslation()) <= 1
    // || AngleMath.getDelta(limeLight.getRobotYaw(),
    // poseEstimator.getEstimatedPosition().getRotation().getDegrees()) >= 8)
    // poseEstimator.addVisionMeasurement(limeLight.getPose(),
    // limeLight.getLastReceiveTime());
    // }
    // }
}
