//package teamcode.vision;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import java.util.Queue;
//import java.util.LinkedList;
//
//
////    //TODO: fill with accurate coordinates for possible duck(triangle) positions
//
//    public class visionRL {
//        OpMode opMode;
//        OpenCvCamera camera;
//        CustomPipeline pipeline;
//
//        private final Point RED_LEFT_MIDDLE_TL = new Point(100, 120);
//        private final Point RED_LEFT_MIDDLE_BR = new Point(150, 150);
//        private final Point RED_LEFT_RIGHT_TL = new Point(230, 120);
//        private final Point RED_LEFT_RIGHT_BR = new Point(280, 150);
//
//        private Point middleTL;
//        private Point middleBR;
//        private Point leftTL;
//        private Point leftBR;
//        region_center_green = green.submat(new Rect(RED_LEFT_MIDDLE_TL, RED_LEFT_MIDDLE_BR));
//        green_avg = (int) Core.mean(region_center_green).val[0];
//
//        private int middleBoxGreen;
//        private int leftBoxGreen;
//        private boolean show_value = true;
//        private static final int NUM_READINGS = 3; // Number of readings to average
//        private Queue<Integer> middleBoxReadings = new LinkedList<>();
//        private Queue<Integer> leftBoxReadings = new LinkedList<>();
//
//        public enum trianglyLocation {
//            LEFT, RIGHT, MIDDLE
//        }
//    public visionRL(OpMode op) {
//        opMode = op;
//        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "camera1"), cameraMonitorViewId);
//
//        pipeline = new CustomPipeline();
//        camera.openCameraDevice();
//        camera.setPipeline(pipeline);
//        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//
//        middleTL = RED_LEFT_RIGHT_TL;
//        middleBR = RED_LEFT_RIGHT_BR;
//        leftTL = RED_LEFT_MIDDLE_TL;
//        leftBR = RED_LEFT_MIDDLE_BR;
//    }
//        public void stopStreaming() {
//            camera.stopStreaming();
//        }
//
//        private int getAverage(Queue<Integer> readings) {
//            int sum = 0;
//            for (int reading : readings) {
//                sum += reading;
//            }
//            return readings.isEmpty() ? 0 : sum / readings.size();
//        }
//        public trianglyLocation mostTrianglyArea() {
//            int significantDifference = 5; // Lowered threshold for significant difference
//            int baseGreenValue = 124; // Base value for detecting presence of object
//
//            int middleBoxAvg = getAverage(middleBoxReadings);
//            int leftBoxAvg = getAverage(leftBoxReadings);
//
//            // Compare each box's average green value against the base value
//            boolean middleHasObject = middleBoxAvg > baseGreenValue;
//            boolean leftHasObject = leftBoxAvg > baseGreenValue;
//
//            trianglyLocation detectedLocation = trianglyLocation.RIGHT; // Default to right
//
//            if (middleHasObject || leftHasObject) {
//                // Determine if the difference between the boxes is significant
//                if (Math.abs(middleBoxAvg - leftBoxAvg) <= significantDifference) {
//                    // When difference is not significant, choose the box with slightly higher value
//                    detectedLocation = (middleBoxAvg >= leftBoxAvg) ? trianglyLocation.MIDDLE : trianglyLocation.LEFT;
//                } else {
//                    // When difference is significant, choose the box with significantly higher value
//                    detectedLocation = (middleBoxAvg > leftBoxAvg) ? trianglyLocation.MIDDLE : trianglyLocation.LEFT;
//                }
//            }
//
//            // Add telemetry for color values and detected location
//            opMode.telemetry.addData("Middle Box Avg Green", middleBoxAvg);
//            opMode.telemetry.addData("Left Box Avg Green", leftBoxAvg);
//            opMode.telemetry.addData("Detected Location", detectedLocation);
//            opMode.telemetry.update();
//
//            return detectedLocation;
//        }
//
//
//
//
//
//        class CustomPipeline extends OpenCvPipeline {
//            @Override
//            public Mat processFrame(Mat input) {
//                // Update readings for each box
//                updateReadings(middleBoxReadings, getAverageGreen(input, middleTL, middleBR));
//                updateReadings(leftBoxReadings, getAverageGreen(input, leftTL, leftBR));
//
//                int thickness = 3;
//                Scalar color = new Scalar(0, 255, 0); // Green color for the rectangle
//
//                trianglyLocation position = mostTrianglyArea();
//                if (position == trianglyLocation.LEFT) {
//                    Imgproc.rectangle(input, leftTL, leftBR, color, thickness); // Highlight left box
//                } else if (position == trianglyLocation.MIDDLE) {
//                    Imgproc.rectangle(input, middleTL, middleBR, color, thickness); // Highlight middle box
//                }
//
//
//                return input;
//            }
//
//        }
//            private void updateReadings(Queue<Integer> readings, int newReading) {
//                if (readings.size() >= NUM_READINGS) {
//                    readings.poll(); // Remove oldest reading
//                }
//                readings.offer(newReading); // Add new reading
//            }
//
//            private int getAverageGreen(Mat mat, Point topLeft, Point bottomRight) {
//                int sumGreen = 0;
//                int pixelCount = 0;
//
//                for (int y = (int) topLeft.y; y < bottomRight.y; y++) {
//                    for (int x = (int) topLeft.x; x < bottomRight.x; x++) {
//                        sumGreen += mat.get(y, x)[1]; // 1 is the index for Green in BGR
//                        pixelCount++;
//                    }
//                }
//
//                return (pixelCount > 0) ? (sumGreen / pixelCount) : 0;
//            }
//
//
//        public void setTelemShow(boolean show) {
//            this.show_value = show;
//        }
//    }