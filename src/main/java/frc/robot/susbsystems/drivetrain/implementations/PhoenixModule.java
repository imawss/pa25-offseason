package frc.robot.susbsystems.drivetrain.implementations;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.google.flatbuffers.Constants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.constants.ModuleConstants;
import frc.robot.susbsystems.drivetrain.interfaces.ModuleInterface;

public class PhoenixModule implements ModuleInterface {

    private TalonFX driveMotor;
    private TalonFXConfiguration driveConfig;

    private TalonFX steerMotor;
    private TalonFXConfiguration steerConfig;

    private CANcoder steerEncoder;
    private CANcoderConfiguration steerEncoderConfig;

    public PhoenixModule(int driveId, int steerId, int encoderId, 
            double wheelOffsetRadians, boolean driveInverted,
            boolean steerInverted, boolean encoderReversed) {

        this.driveMotor = new TalonFX(driveId);
        driveConfig.Feedback.SensorToMechanismRatio = ModuleConstants.DRIVE_GEAR_RATIO;
        driveConfig.Feedback.RotorToSensorRatio = 1.0;
        //driveConfig.MotorOutput.Inverted = driveInverted ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive;

        this.steerMotor = new TalonFX(steerId);
        this.steerEncoder = new CANcoder(encoderId);
    }

    @Override
    public void setDriveMotorVoltage(Voltage voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDriveMotorVoltage'");
    }

    @Override
    public void setAngleMotorVoltage(Voltage voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAngleMotorVoltage'");
    }

    @Override
    public void setModuleState(SwerveModuleState desiredState) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setModuleState'");
    }

    @Override
    public SwerveModuleState getModuleState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getModuleState'");
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getModulePosition'");
    }

    @Override
    public Angle getRawWheelPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRawWheelPosition'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

}
