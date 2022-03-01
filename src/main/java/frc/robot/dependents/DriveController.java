package frc.robot.dependents;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
