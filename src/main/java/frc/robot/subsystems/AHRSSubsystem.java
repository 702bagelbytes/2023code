package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AHRSSubsystem extends SubsystemBase {
    private final AHRS ahrs = new AHRS();

    public AHRSSubsystem() {
    }

    public float getRoll() {
        return ahrs.getRoll();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("Pitch", ahrs.getPitch());
        SmartDashboard.putNumber("Roll", ahrs.getRoll());
    }
}
