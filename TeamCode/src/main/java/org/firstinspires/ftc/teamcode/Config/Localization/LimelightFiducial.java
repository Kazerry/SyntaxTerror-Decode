package org.firstinspires.ftc.teamcode.Config.Localization;

import static org.firstinspires.ftc.teamcode.Config.RobotConstants.kP;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.kI;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.kD;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

public class LimelightFiducial {

    private Limelight3A limelight;
    private Follower follower;
    public static double TX;
    public static double TY;
    public static double TA;
    public static double AprilTagDistance;
    public static double startHeading;
    public static double turretPower;
    public static List<LLResultTypes.FiducialResult> fiducialResults;
    public static LLResult result;
    public static List<LLResultTypes.FiducialResult> fiducials;
    private PIDController pid;
    /**
     * Initialize the Limelight
     */
    public void LimelightInit(Limelight3A limelight) { //, Follower follower
        this.limelight = limelight;
        //this.follower = follower;
        limelight.start();
        pid = new PIDController(kP, kI, kD);
    }
    /**
     * Get the latest pose from the Limelight
     * Update Turret Power
     * Use this to update in the background
     *
     * @return
     * Calculates limelight fiducials
     * Returns a double for turretpower and calculates the PID
     */
    public double getLimePose() {
        result = limelight.getLatestResult();

        fiducials = result.getFiducialResults();

        if (result == null || !result.isValid() || fiducials.isEmpty()) {
            turretPower = 0;
            pid.reset();
            return 0;
        } else {
            int id = fiducials.get(0).getFiducialId(); // The ID number of the fiducial
            TX = fiducials.get(0).getTargetXDegrees(); // Where it is (left-right)
            TY = fiducials.get(0).getTargetYDegrees(); // Where it is (up-down)
            AprilTagDistance = fiducials.get(0).getCameraPoseTargetSpace().getPosition().y;

            turretPower = pid.calculate(TX);
            //turretPower = Math.max(-1, Math.min(1, turretPower)); //putting limits on maximum and minimum power
            return turretPower;
        }
    }
}