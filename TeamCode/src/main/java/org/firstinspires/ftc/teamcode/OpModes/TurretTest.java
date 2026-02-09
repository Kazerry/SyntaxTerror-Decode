package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Config.Localization.LimelightFiducial.TX;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Config.Localization.LimelightFiducial;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@TeleOp
public class TurretTest extends OpMode {
    private Limelight3A limelight;
    private LimelightFiducial LimeInit;
    private DcMotorEx TurretMotor;
    private PIDController pid;
    private double turretPower;
    public static double kP = 0.3;
    public static double kI = 0.00;
    public static double kD = 0.0001;
    public static double DeadDeg = 1.5;
    private double setpoint = 0;
    private Follower follower;
    private Pose startingPose;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        TurretMotor = hardwareMap.get(DcMotorEx.class, "TurretMotor");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        LimeInit = new LimelightFiducial();
        LimeInit.LimelightInit(limelight);
        pid = new PIDController(kP, kI, kD);

        TurretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        TurretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        TurretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start(){
        follower.startTeleopDrive();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                -gamepad1.right_stick_x,
                true
        );
    }

    @Override
    public void loop(){
        LimeInit.getLimePose();
        follower.update();

        com.qualcomm.hardware.limelightvision.LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            TurretMotor.setPower(gamepad2.left_stick_x);
            return;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            TurretMotor.setPower(gamepad2.left_stick_x);
            return;
        }
        double tx = fiducials.get(0).getTargetXDegrees();
        turretPower = pid.calculate(tx);
        turretPower = Math.max(-0.6, Math.min(0.6, turretPower));
        //turretPower *= Math.min(1, Math.abs(tx) / deadband); //Output taper
        TurretMotor.setPower(turretPower);

        //Stop if within deadband
        if (Math.abs(tx) < DeadDeg) {
            TurretMotor.setPower(0);
            pid.reset();
        }

        telemetry.addData("TX in degrees", tx);
        telemetry.addData("kP Proportional", kP);
        telemetry.addData("kI Integral", kI);
        telemetry.addData("kD Derivative", kD);
        telemetry.addData("Turret Power", turretPower);
        telemetry.update();
    }
}
