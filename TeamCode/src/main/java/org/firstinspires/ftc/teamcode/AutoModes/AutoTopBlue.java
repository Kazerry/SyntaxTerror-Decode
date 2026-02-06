package org.firstinspires.ftc.teamcode.AutoModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueTopAuto", group = "Autonomous")
public class AutoTopBlue extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int actionState;

    private DcMotorEx IntakeMotor;
    private DcMotorEx TurretMotor;
    private DcMotorEx OTMotor;

    private CRServo tServo;

    private final Pose startPose = new Pose(54, 7.5, Math.toRadians(90));
    private final Pose intakePose = new Pose(10,8);
    private final Pose shootPose = new Pose(70,20);

    private Path intake, shoot1, return1;

    public void buildPaths() {
        intake = new Path(new BezierLine(startPose,intakePose));
        intake.setConstantHeadingInterpolation(Math.toRadians(90));

        shoot1 = new Path(new BezierLine(intakePose,shootPose));
        shoot1.setConstantHeadingInterpolation(Math.toRadians(90));

        return1 = new Path(new BezierCurve(shootPose, new Pose(40, 7), intakePose));
        return1.setConstantHeadingInterpolation(Math.toRadians(90));
    }
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                setActionState(0);
                follower.followPath(intake);
                setPathState(1);
                break;
            case 1:
                if(follower.atParametricEnd()) {
                    setActionState(1);
                    follower.setMaxPower(1);
                    follower.followPath(shoot1);
                    setPathState(2);
                }
                break;
            case 2:
                if(follower.atParametricEnd()) {
                    setActionState(2);
                    if (actionTimer.getElapsedTime() >= 5) {
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(follower.atParametricEnd()) {
                    setActionState(3);
                    follower.followPath(return1);
                    setPathState(4);
                }
                break;
            case 4:
                if(follower.atParametricEnd()){

                }
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
                IntakeMotor.setPower(1);
                break;
            case 1:
                IntakeMotor.setPower(0);
                break;
            case 2:
                actionTimer.resetTimer();
                //Outtake stuff!
                break;
            case 3:
                actionTimer.resetTimer();
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
        IntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        OTMotor = hardwareMap.get(DcMotorEx .class, "OTMotor");
        OTMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        OTMotor.setDirection(DcMotorEx.Direction.FORWARD);
        OTMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        tServo = hardwareMap.get(CRServo.class, "tServo");

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

    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        if (pathState == 0 && pathTimer.getElapsedTime() >= 1.3){
            follower.setMaxPower(0.3);
        }
    }
}
