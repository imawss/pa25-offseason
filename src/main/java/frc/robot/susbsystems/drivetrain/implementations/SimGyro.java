package frc.robot.susbsystems.drivetrain.implementations;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.susbsystems.drivetrain.interfaces.GyroInterface;

public class SimGyro implements GyroInterface{

  private final GyroSimulation gyroSimulation;

  public SimGyro(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public Rotation2d getAngle() {
    return gyroSimulation.getGyroReading();
  }

  @Override
  public void setAngle(Rotation2d angle) {
    gyroSimulation.setRotation(angle);
  }
  
}