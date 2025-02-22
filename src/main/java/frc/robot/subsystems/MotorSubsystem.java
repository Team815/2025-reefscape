package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
    private final SparkMax motor;


    public MotorSubsystem(SparkMax motor) {
        this.motor = motor;
    }

    public Command run(double speed) {
        return startEnd(
            () -> motor.set(speed),
            () -> motor.set(0));
    }
}
