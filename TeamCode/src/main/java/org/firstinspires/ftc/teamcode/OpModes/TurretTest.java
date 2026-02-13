package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Config.Localization.LimelightFiducial.TX;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.kP;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.kI;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.kD;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Config.Localization.LimelightFiducial;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@TeleOp
public class TurretTest extends OpMode {
    private Limelight3A limelight;
    private LimelightFiducial LimeInit;
    private DcMotorEx TurretMotor;
    private double setpoint = 0;
    private Follower follower;
    private Pose startingPose;
    private List<LLResultTypes.FiducialResult> fiducials;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        TurretMotor = hardwareMap.get(DcMotorEx.class, "TurretMotor");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); //1 blue 0 red
        LimeInit = new LimelightFiducial();
        LimeInit.LimelightInit(limelight);

        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        TurretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        TurretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop(){
        telemetry.addData("Turret Position",TurretMotor.getCurrentPosition());
        telemetry.update();
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
        double turretPower = LimeInit.getLimePose();
        follower.update();

        TurretMotor.setPower(turretPower);

        telemetry.addData("TX in degrees", TX);
        telemetry.addData("kP Proportional", kP);
        telemetry.addData("kI Integral", kI);
        telemetry.addData("kD Derivative", kD);
        telemetry.addData("Turret Power", turretPower);
        telemetry.update();
    }
}
