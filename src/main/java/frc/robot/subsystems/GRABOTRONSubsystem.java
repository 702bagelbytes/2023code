package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GRABOTRONConstants;

public class GRABOTRONSubsystem extends SubsystemBase {
    private final Compressor phCompressor = new Compressor(GRABOTRONConstants.kRevPneumaticsHubId,
            PneumaticsModuleType.REVPH);
    private final DoubleSolenoid grabSolenoid = new DoubleSolenoid(GRABOTRONConstants.kRevPneumaticsHubId,
            PneumaticsModuleType.REVPH, GRABOTRONConstants.kExtendSolenoid, GRABOTRONConstants.kRetractSolenoid);

    public GRABOTRONSubsystem() {
        // phCompressor.disable();
        phCompressor.enableDigital();
        grabSolenoid.set(Value.kReverse);
        SmartDashboard.putString("Pressure", String.format("%.2f",
                phCompressor.getPressure()));
    }

    public DoubleSolenoid getSolenoid() {
        return grabSolenoid;
    }

    public void toggle() {
        grabSolenoid.toggle();
    }

    public Command toggleCommand() {
        return this.runOnce(this::toggle);
    }
}