package org.firstinspires.ftc.teamcode.Config.Localization;

import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
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
    public static List<LLResultTypes.FiducialResult> fiducialResults;
    /**
     * Initialize the Limelight
     */
    public void LimelightInit(Limelight3A limelight) { //, Follower follower
        this.limelight = limelight;
        //this.follower = follower;
        limelight.start();
        limelight.pipelineSwitch(0);
    }
    /** Get the latest pose from the Limelight
     * Use this to update in the background
     */
    public void getLimePose() {
        //LLResult botpose = limelight.getLatestResult(); //probably dont need this

        List<FiducialResult> fiducials = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double TX = fiducial.getTargetXDegrees(); // Where it is (left-right)
            double TY = fiducial.getTargetYDegrees(); // Where it is (up-down)
            double AprilTagDistance = fiducial.getCameraPoseTargetSpace().getPosition().y;
        }

       /* could use this one
       LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double TX = result.getTx(); // How far left or right the target is (degrees)
            double TY = result.getTy(); // How far up or down the target is (degrees)
            double TA = result.getTa(); // How big the target looks (0%-100% of the image)
        }
        fiducialResults = limelight.getLatestResult().getFiducialResults();
        */
    }
}