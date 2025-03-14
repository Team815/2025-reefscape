package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClawThingy extends MotorSubsystem {
    private final DigitalInput coralSensor;

    private final DigitalInput algaeSensor;

    public ClawThingy(SparkMax motor, DigitalInput coralSensor, DigitalInput algaeSensor) {
        super(motor);
        this.coralSensor = coralSensor;
        this.algaeSensor = algaeSensor;
    }

    public boolean hasCoral(){
        return coralSensor.get();
    }

    public boolean hasAlgae(){
        return algaeSensor.get();
    }

    public Command receiveCoral(){
        return run(-0.5).withDeadline(Commands.waitUntil(this::hasCoral));
    }
}
