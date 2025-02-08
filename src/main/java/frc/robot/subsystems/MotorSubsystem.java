package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
    private final TalonFX motor;


    public MotorSubsystem(TalonFX motor) {
        this.motor = motor;
    }

    public Command run(double speed) {
        return startEnd(
            () -> motor.set(speed),
            () -> motor.set(0));
    }
}
