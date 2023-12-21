package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final PIDController turningPID;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorID,
            int turnMotorID,
            boolean driveMotorReversed,
            boolean turningMotorReversed,
            int absoluteEncoderID,
            double absoluteEncoderOffset,
            boolean absEncoderReversed) {
        absoluteEncoderOffsetRad = absoluteEncoderOffset;
        absoluteEncoderReversed = absEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderID);

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(Constants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(Constants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(Constants.kTurningEncoderRot2Rad);
        driveEncoder.setVelocityConversionFactor(Constants.kTurningEncoderRPM2RadPerSec);

        turningPID = new PIDController(Constants.kPTurning, Constants.kITurning, Constants.kDTurning);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = (absoluteEncoder.getSupplyVoltage().getValueAsDouble() / RobotController.getVoltage5V());
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			driveEncoder.getPosition(),
			new Rotation2d(turnEncoder.getPosition()));
	}

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(turningPID.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
