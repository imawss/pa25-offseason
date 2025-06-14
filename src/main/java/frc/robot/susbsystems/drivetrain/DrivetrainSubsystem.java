package frc.robot.susbsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.susbsystems.drivetrain.enums.DriveType;
import frc.robot.susbsystems.drivetrain.interfaces.GyroInterface;
import frc.robot.susbsystems.drivetrain.interfaces.ModuleInterface;
import frc.robot.utils.Logger;

public class DrivetrainSubsystem extends SubsystemBase {

    private final SwerveDrivePoseEstimator estimator;

    private final GyroInterface gyro;

    private final ModuleInterface frontLeft, frontRight, backLeft, backRight;

    private final PPHolonomicDriveController driveController;

    public DrivetrainSubsystem(
            GyroInterface gyro,
            ModuleInterface frontLeft,
            ModuleInterface frontRight,
            ModuleInterface backLeft,
            ModuleInterface backRight) {

        this.gyro = gyro;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.estimator = new SwerveDrivePoseEstimator(
                DrivetrainConstants.KINEMATICS,
                gyro.getAngle(),
                getModulePositions(),
                RobotConstants.INITIAL_POSE);

        this.driveController = new PPHolonomicDriveController(
                AutoConstants.DRIVE_PID,
                AutoConstants.ANGLE_PID);

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> this.drive(speeds),
                this.driveController,
                RobotConstants.ROBOT_CONFIG,
                () -> false,
                this);

        AutoBuilder.buildAutoChooser();
    }

    @Override
    public void periodic() {

        this.estimator.update(this.getRotation2d(), this.getModulePositions());

        Logger.log("Drivetrain/Poses/OdometryPose", this.getPose());

        Logger.log("Drivetrain/Modules/FrontLeft/State", this.frontLeft.getModuleState());
        Logger.log("Drivetrain/Modules/FrontRight/State", this.frontRight.getModuleState());
        Logger.log("Drivetrain/Modules/BackLeft/State", this.backLeft.getModuleState());
        Logger.log("Drivetrain/Modules/BackRight/State", this.backRight.getModuleState());

        Logger.log("Drivetrain/Modules/ModuleStates", this.getModuleStates());

        Logger.log("Drivetrain/Gyro/Angle", gyro.getAngle().getDegrees());
    }

    public Rotation2d getRotation2d() {
        return gyro.getAngle();
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, RobotConstants.MAX_SPEED);

        this.frontLeft.setModuleState(moduleStates[0]);
        this.frontRight.setModuleState(moduleStates[1]);
        this.backLeft.setModuleState(moduleStates[2]);
        this.backRight.setModuleState(moduleStates[3]);

        Logger.log("Drivetrain/Modules/DesiredModuleStates", moduleStates);
    }

    public void drive(double xSpeed, double ySpeed, double rSpeed, DriveType driveType) {

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rSpeed);

        if (driveType == DriveType.RobotRelative) {
            this.drive(speeds);
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getRotation2d());

            this.drive(speeds);
        }
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(speeds);

        this.setModuleStates(moduleStates);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DrivetrainConstants.KINEMATICS.toChassisSpeeds(this.getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                this.frontLeft.getModuleState(),
                this.frontRight.getModuleState(),
                this.backLeft.getModuleState(),
                this.backRight.getModuleState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                this.frontLeft.getModulePosition(),
                this.frontRight.getModulePosition(),
                this.backLeft.getModulePosition(),
                this.backRight.getModulePosition(),
        };
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        estimator.resetPose(pose);
    }

    public ModuleInterface[] getModules() {
        return new ModuleInterface[] {
                this.frontLeft,
                this.frontRight,
                this.backLeft,
                this.backRight
        };
    }
}