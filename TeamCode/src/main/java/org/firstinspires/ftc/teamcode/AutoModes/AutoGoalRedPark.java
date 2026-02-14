package org.firstinspires.ftc.teamcode.AutoModes;

import static org.firstinspires.ftc.teamcode.Config.RobotConstants.autoPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Config.Localization.LimelightFiducial;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RedParkGoal", group = "Autonomous")
public class AutoGoalRedPark extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int actionState;

    private DcMotorEx IntakeMotor;
    private DcMotorEx TurretMotor;
    private DcMotorEx OTMotor;
    private CRServo tServo;

    private Limelight3A limelight;
    private LimelightFiducial LimeInit;

    private final Pose startPose = new Pose(90, 7.5, Math.toRadians(36));
    private final Pose parkPose = new Pose(40.5,33.5);

    private Path park;

    public void buildPaths() {

        park = new Path(new BezierLine(startPose, parkPose));
        park.setLinearHeadingInterpolation(Math.toRadians(36),Math.toRadians(90));
    }
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                setActionState(0);
                follower.followPath(park);
                setPathState(1);
                break;
        }
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousActionUpdate(){
        switch (actionState){
            case 0:
                actionTimer.resetTimer();
                OTMotor.setPower(0);
                tServo.setPower(0);
                IntakeMotor.setPower(0);
                //turn things off
                break;
            case 1:
                actionTimer.resetTimer();
                tServo.setPower(10);
                IntakeMotor.setPower(10);
                //Intake stuff
                break;
            case 2:
                actionTimer.resetTimer();
                tServo.setPower(10);
                OTMotor.setPower(10);
                //Outtake stuff!
                break;
            case 3:
                actionTimer.resetTimer();
                OTMotor.setPower(10);
                tServo.setPower(10);
                IntakeMotor.setPower(10);
                //Everything on
                break;
            case 4:
                actionTimer.resetTimer();
                break;
        }
    }
    public void setActionState(int aState){
        actionState = aState;
        actionTimer.resetTimer();
    }
    @Override
    public void init() {
        TurretMotor = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        TurretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        TurretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        TurretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        IntakeMotor = hardwareMap.get(DcMotorEx .class, "IntakeMotor");
        IntakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        OTMotor = hardwareMap.get(DcMotorEx .class, "OTMotor");
        OTMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        OTMotor.setDirection(DcMotorEx.Direction.FORWARD);
        OTMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tServo = hardwareMap.get(CRServo.class, "tServo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); //1 blue 0 red
        LimeInit = new LimelightFiducial();
        LimeInit.LimelightInit(limelight);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    public void stop() {
        autoPose = follower.getPose();
    }

    @Override
    public void loop(){
        follower.update();
        double turretPower = LimeInit.getLimePose();
        TurretMotor.setPower(turretPower);
        autonomousPathUpdate();
        autonomousActionUpdate();
        telemetry.addData("actionTimer",actionTimer.getElapsedTimeSeconds());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
    }
}
