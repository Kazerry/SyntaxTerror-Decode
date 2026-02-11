package org.firstinspires.ftc.teamcode.OpModes;
import static org.firstinspires.ftc.teamcode.Config.Localization.LimelightFiducial.TX;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.kD;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.kI;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.kP;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Config.Localization.LimelightFiducial;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
import java.lang.Math.*;

@Configurable
@TeleOp
public class TurretOPBlue extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    //private DcMotorEx TurretMotor;
    private DcMotorEx IntakeMotor;
    private DcMotorEx OTMotor;
    private Limelight3A limelight;
    private LimelightFiducial LimeInit;
    private DcMotorEx TurretMotor;

    private CRServo tServo;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); //1 blue 0 red
        LimeInit = new LimelightFiducial();
        LimeInit.LimelightInit(limelight);

        TurretMotor = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        TurretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        TurretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        TurretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        OTMotor = hardwareMap.get(DcMotorEx.class, "OTMotor");
        OTMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        OTMotor.setDirection(DcMotorEx.Direction.FORWARD);
        OTMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        IntakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        tServo = hardwareMap.get(CRServo.class, "tServo");

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        double turretPower = LimeInit.getLimePose();
        follower.update();

        TurretMotor.setPower(turretPower);

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false // Robot Centric
                );
                follower.setMaxPower(1);
            }
                //This is how it looks with slowMode on
            else {
                follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );
            follower.setMaxPower(0.5);
            }
        }
/*
        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
*/
        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //TurretMotor.setPower(gamepad2.right_stick_x*0.25);
        if (gamepad2.right_stick_y >= 0.5 || gamepad1.right_trigger >= 0.5) {
            OTMotor.setPower(1);
            tServo.setPower(1);
        } else {
            OTMotor.setPower(0);
            tServo.setPower(0);
        }
        if (gamepad2.left_stick_y >= 0.5 || gamepad1.left_trigger >= 0.5) {
            IntakeMotor.setPower(100);
            tServo.setPower(1);
        } else {
            IntakeMotor.setPower(0);
            tServo.setPower(0);
        }
        telemetry.addData("TX in degrees", TX);
        telemetry.addData("kP Proportional", kP);
        telemetry.addData("kI Integral", kI);
        telemetry.addData("kD Derivative", kD);
        telemetry.addData("Turret Power", turretPower);
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetry.update();
    }
}