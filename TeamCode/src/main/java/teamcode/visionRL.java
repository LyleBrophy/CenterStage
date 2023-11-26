package teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class visionRL {
    OpMode opMode;
    OpenCvCamera camera1;
    AddBoxesPipeline pipeline;
    private final Point centerBox_topLeft = new Point(150, 133);
    private final Point centerBox_bottomRight = new Point(190, 174);

        private final Point leftBox_topLeft = new Point(73,100);
    private final Point leftBox_bottomRight = new Point(73,150);

    private final Point rightBox_topLeft = new Point(230, 100);
    private final Point rightBox_bottomRight = new Point(230, 150);
    Mat YCrCb = new Mat();
    Mat red = new Mat();
    Mat green = new Mat();
    Mat blue = new Mat();
    Mat region_center_red, region_center_green, region_center_blue;
    Mat region_left_red, region_left_green, region_left_blue;
    Mat region_right_red,region_right_green,region_right_blue;
    int red_avg, green_avg, blue_avg;
    public boolean one = false;
    public boolean two = false;
    public boolean three = false;

    public visionRL(OpMode op) {
        opMode = op;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "camera1"), cameraMonitorViewId);
        pipeline = new AddBoxesPipeline();
        camera1.openCameraDevice();
        camera1.setPipeline(pipeline);
        camera1.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public void stopStreaming() {
        camera1.stopStreaming();
    }

    //Add boxes to the image display
    class AddBoxesPipeline extends OpenCvPipeline {
        void inputToColors(Mat input) {
            //Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(input, red, 0);
            Core.extractChannel(input, green, 1);
            Core.extractChannel(input, blue, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToColors(firstFrame);
            region_center_red = red.submat(new Rect(centerBox_topLeft, centerBox_bottomRight));
            region_center_green = green.submat(new Rect(centerBox_topLeft, centerBox_bottomRight));
            region_center_blue = blue.submat(new Rect(centerBox_topLeft, centerBox_bottomRight));

            region_left_red = red.submat(new Rect(leftBox_topLeft, leftBox_bottomRight));
            region_left_green = green.submat(new Rect(leftBox_topLeft, leftBox_bottomRight));
            region_left_blue = blue.submat(new Rect(leftBox_topLeft, leftBox_bottomRight));

            region_right_red = red.submat(new Rect(rightBox_topLeft, rightBox_bottomRight));
            region_right_green = green.submat(new Rect(rightBox_topLeft, rightBox_bottomRight));
            region_right_blue = blue.submat(new Rect(rightBox_topLeft, rightBox_bottomRight));
        }

        //This processes the visual output on the screen
        @Override
        public Mat processFrame(Mat input) {
            inputToColors(input);
            red_avg = (int) Core.mean(region_center_red).val[0];
            green_avg = (int) Core.mean(region_center_green).val[0];
            blue_avg = (int) Core.mean(region_center_blue).val[0];
            opMode.telemetry.addData("RED: ", red_avg);
            opMode.telemetry.addData("GREEN: ", green_avg);
            opMode.telemetry.addData("BLUE: ", blue_avg);
            opMode.telemetry.update();
            if (red_avg >= 115) {
                one = true;
                two = false;
                three = false;
            } else if (blue_avg >= 120) {
                one = false;
                two = false;
                three = true;
            }
            //opMode.telemetry.addData("boxRight: ", right_avg);
            //opMode.telemetry.update();
//            else if(right_avg <= left_avg && right_avg <= center_avg){
            else {
                one = false;
                two = true;
                three = false;
            }
            Scalar red = new Scalar(0, 0, 0);
            Scalar green = new Scalar(0, 0, 0);
            Scalar blue = new Scalar(0, 0, 0);
            Imgproc.rectangle(input, centerBox_topLeft, centerBox_bottomRight, blue, 3);
            Imgproc.rectangle(input, leftBox_topLeft, leftBox_bottomRight, red, 3);
            Imgproc.rectangle(input, rightBox_topLeft, rightBox_bottomRight, red, 3);
//            if (one) {
//                red = new Scalar(255, 0, 0);
//                Imgproc.rectangle(input, centerBox_topLeft, centerBox_bottomRight, red, thickness);
//            } else if (two) {
//                green = new Scalar(0, 255, 0);
//                Imgproc.rectangle(input, centerBox_topLeft, centerBox_bottomRight, green, thickness);
//            } else {
//                blue = new Scalar(0, 0, 255);
//                Imgproc.rectangle(input, centerBox_topLeft, centerBox_bottomRight, blue, thickness);
//            }
            return input;
        }

    }
}