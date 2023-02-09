package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class LeftAuto extends LinearOpMode {

    //amount of grab the claw does //go higher for more grab, go less for less grab
    private double grab = 1;
    //the prep pivot amount //to clear the pole when extending, keep this number lower than the score pivot amount // decrease to be further from the pole, increase to be closer
    private int preppivot = 1220;
    //the score pivot amount //increase to be further over the pole, decrease to be less over the pole
    private int scorepivot = 1280-5;
    //the amount the slide extends when scoring //increase to reach out more, decrease to reach out less
    private int scoreslide = 695;

    // arm angle to grab //increase to be higher off ground, decrease to be lower
    int entryVal1 = 405;
    int entryVal2 = 390;
    int entryVal3 = 360+5;
    int entryVal4 = 350+2;
    int EntryVal5 = 313+10;

    //the amount the slide reaches //increase/decrease by small numbers, increase to reach further, decrease to reach less far
    private int reach1 = 670;
    private int reach2 = 680;
    private int reach3 = 685;
    private int reach4 = 695;
    private int reach5 = 695;

    double servoinpulses = 20.0 / 41793;
    public DcMotor Arm;
    public DcMotor slide;
    public Servo wrist;
    public Servo claw;
    public DcMotor leftRear;
    public DcMotor rightRear;
    public DcMotor leftFront;
    public DcMotor rightFront;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public int finalx;
    public int finaly;
    public int finalh;

    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //tag ID's of sleeve
    int left = 1;
    int middle = 2;
    int right = 3;








    AprilTagDetection tagOfInterest = null;







    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        Arm = hardwareMap.dcMotor.get("pivot");
        slide = hardwareMap.dcMotor.get("slide");
        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");



        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slide.setTargetPosition(0);
        slide.setPower(1);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        claw.setPosition(grab);
        wrist.setPosition(0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35,-64, Math.toRadians(90));


        drive.setPoseEstimate(startPose);

        //waitForStart();
        //wait for start SCAN
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        //trajectory stuff
        TrajectorySequence trajseq = null;
        //TrajectorySequence traj2 = null;
        {
            trajseq = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-35, -22, (Math.toRadians(135))))
                    .lineToLinearHeading(new Pose2d(-31, -3.7, (Math.toRadians(189))))

                    .addDisplacementMarker(20, () -> {
                    slide.setTargetPosition(0);
                    slide.setPower(1);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    //FINISH LINE
                    .UNSTABLE_addTemporalMarkerOffset(-2.7,() -> {
                        armset(preppivot+5,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        slideset(scoreslide,1);

                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3,() -> {
                        armset(scorepivot+4,1);

                    })
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        claw.setPosition(0.5);
                    })
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        armset(1190,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                        armset(entryVal1,1);

                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                        slideset(0,1);
                    })
                    .waitSeconds(1)
                    //
                    //
                    //

                    //GRAB ONE
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        slideset(reach1,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                        claw.setPosition(grab);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.4,() -> {
                        armset(475,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.9,() -> {
                        slideset(0,1);
                    })
                    .waitSeconds(3.2)//issue
                    //
                    //
                    //
                    //FINISH LINE
                    .UNSTABLE_addTemporalMarkerOffset(-1.2,() -> {
                        armset(preppivot-5,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.7,() -> {
                        slideset(scoreslide,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        armset(scorepivot+4,1);

                    })
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                        claw.setPosition(0.5);
                    })
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        armset(1190,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                        armset(entryVal2,1);

                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                        slideset(0,1);
                    })
                    .waitSeconds(1)

                    //
                    //
                    //

                    //GRAB TWO
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        slideset(reach2,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                        claw.setPosition(grab);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.4,() -> {
                        armset(475,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.9,() -> {
                        slideset(0,1);
                    })
                    .waitSeconds(3.2)
                    //
                    //
                    //
                    //FINISH
                    .UNSTABLE_addTemporalMarkerOffset(-1.2,() -> {
                        armset(preppivot,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.7,() -> {
                        slideset(scoreslide,1);

                    })
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        armset(scorepivot+4,1);

                    })
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                        claw.setPosition(0.5);
                    })
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        armset(1190,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                        armset(entryVal3,1);

                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                        slideset(0,1);
                    })
                    .waitSeconds(1)
                    //
                    //
                    //

                    //GRAB THREE
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        slideset(reach3,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                        claw.setPosition(grab);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.4,() -> {
                        armset(475,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.9,() -> {
                        slideset(0,1);
                    })
                    .waitSeconds(3.2)
                    //
                    //
                    //
                    //FINISH
                    .UNSTABLE_addTemporalMarkerOffset(-1.2,() -> {
                        armset(preppivot,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.7,() -> {
                        slideset(scoreslide,1);

                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                        armset(scorepivot+4,1);

                    })
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                        claw.setPosition(0.5);
                    })
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        armset(1190,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                        armset(entryVal4,1);

                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                        slideset(0,1);
                    })
                    .waitSeconds(1)
                    //
                    //
                    //

                    //GRAB FOUR
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        slideset(reach4,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                        claw.setPosition(grab);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.4,() -> {
                        armset(475,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.9,() -> {
                        slideset(0,1);
                    })
                    .waitSeconds(3.2)
                    //
                    //
                    //
                    //FINISH LINE
                    .UNSTABLE_addTemporalMarkerOffset(-1.2,() -> {
                        armset(preppivot-10,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.7,() -> {
                        slideset(scoreslide,1);

                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                        armset(scorepivot+4,1);

                    })
                    .waitSeconds(0.7)
                    .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                        claw.setPosition(0.5);
                    })
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        armset(1190,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                        armset(0,1);
                        //should be 313 but if park, make 0
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                        slideset(0,1);
                    })
                    .waitSeconds(1)
                    //
                    //
                    //
                    /*
                    //GRAB FIVE
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        slideset(reach5,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                        claw.setPosition(grab);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5,() -> {
                        armset(364,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.9,() -> {
                        slideset(0,1);
                    })
                    .waitSeconds(3.2)
                    //
                    //
                    //
                    //FINISH LINE
                    .UNSTABLE_addTemporalMarkerOffset(-1,() -> {
                        armset(preppivot-5,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.7,() -> {
                        slideset(scoreslide,1);

                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                        armset(scorepivot+5,1);

                    })
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                        claw.setPosition(0.5);
                    })
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        armset(1190,1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                        armset(0,1);

                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.7,() -> {
                        slideset(0,1);

                    })*/







                    .build();

        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == middle) {
            //middle code
            finalx = -36;
            finaly = -12;
            finalh = 0;
            telemetry.addLine("Middle");
            telemetry.update();
        }else if (tagOfInterest.id == left){
            //left code
            finalx = -58;
            finaly = -12;
            finalh = 0;
            telemetry.addLine("left");
            telemetry.update();
        }else if (tagOfInterest.id == right){
            //right code
            finalx = -12;
            finaly = -15;
            finalh = 0;
            telemetry.addLine("right");
            telemetry.update();
        }
        Trajectory straf = drive.trajectoryBuilder(trajseq.end())
                .lineToLinearHeading(new Pose2d(-35,-12,0))
                .build();

        Trajectory park = drive.trajectoryBuilder(straf.end())
                .lineToLinearHeading(new Pose2d(finalx,finaly,Math.toRadians(finalh)))
                .build();



        if (isStopRequested()) return;
        drive.followTrajectorySequence(trajseq);
        drive.followTrajectory(straf);
        drive.followTrajectory(park);


/*
        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()

 */
    }
    private void armset(int pos, double power){
        Arm.setTargetPosition(pos);
        Arm.setPower(power);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(pos*servoinpulses);
    }
    private void slideset(int pos, double power){
        slide.setTargetPosition(pos);
        slide.setPower(power);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
