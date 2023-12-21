package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            Constants.kFrontLeftDriveMotorPort,
            Constants.kFrontLeftTurningMotorPort,
            Constants.kFrontLeftDriveEncoderReversed,
            Constants.kFrontLeftTurningEncoderReversed,
            Constants.kFrontLeftDriveAbsoluteEncoderPort,
            Constants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            Constants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            Constants.kFrontRightDriveMotorPort,
            Constants.kFrontRightTurningMotorPort,
            Constants.kFrontRightDriveEncoderReversed,
            Constants.kFrontRightTurningEncoderReversed,
            Constants.kFrontRightDriveAbsoluteEncoderPort,
            Constants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            Constants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            Constants.kBackLeftDriveMotorPort,
            Constants.kBackLeftTurningMotorPort,
            Constants.kBackLeftDriveEncoderReversed,
            Constants.kBackLeftTurningEncoderReversed,
            Constants.kBackLeftDriveAbsoluteEncoderPort,
            Constants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            Constants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            Constants.kBackRightDriveMotorPort,
            Constants.kBackRightTurningMotorPort,
            Constants.kBackRightDriveEncoderReversed,
            Constants.kBackRightTurningEncoderReversed,
            Constants.kBackRightDriveAbsoluteEncoderPort,
            Constants.kBackRightDriveAbsoluteEncoderOffsetRad,
            Constants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS navx = new AHRS(SerialPort.Port.kMXP, AHRS.SerialDataType.kProcessedData, Constants.navXUpdateRate);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0), new SwerveModulePosition[] {
        frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
    });

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep((1000));
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        navx.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(navx.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(),
			frontRight.getPosition(),
			backLeft.getPosition(),
			backRight.getPosition()
        });
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
