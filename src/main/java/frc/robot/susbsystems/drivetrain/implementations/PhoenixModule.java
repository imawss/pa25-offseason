package frc.robot.susbsystems.drivetrain.implementations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ModuleConstants;
import frc.robot.susbsystems.drivetrain.interfaces.IModuleInterface;

public class PhoenixModule implements IModuleInterface {

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
        driveConfig.MotorOutput.Inverted = driveInverted ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive;

        this.steerMotor = new TalonFX(steerId);
        this.steerEncoder = new CANcoder(encoderId);
    }

    @Override
    public void setDriveMotorVoltage(Voltage voltage) {
        driveMotor.setControl(new VoltageOut(voltage));
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
