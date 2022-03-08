
package frc.robot;
import org.opencv.core.*;

import org.opencv.core.Point;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.awt.AWTException;
import java.awt.BorderLayout;

import java.awt.Container;
import java.awt.Image;
import java.awt.Rectangle;
import java.awt.Robot;
import java.awt.Toolkit;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.BoxLayout;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.videoio.VideoCapture;

import java.util.Timer;
import java.util.TimerTask;

import java.awt.*;
import java.awt.event.*;
import java.net.URL;
import javax.swing.*;

import edu.wpi.first.cameraserver.CameraServer;

public class ThresholdInRange {	
	public static int varForTimer = 0;
    private static int screenCenterX = 317;
    private static int screenCenterY = 240;
    
    //screen size: x= 634, y =  480
    
  //measurement:
    private static double focalLength = 2*320.8; //focal length in pixels
//    private static double ballRadius = 4.5; //cm
    private static double ballRadius = 12.5;
    private static double distanceCameraToBall = 0;
    private static int robotDepth = 0;
    
    private static double depth = ballRadius;
    private static double cameraAngle = 90.0;//change this to another angle from flour
    private static double ballDistance;
    
    private static final String WINDOW_NAME = "Thresholding Operations using inRange demo";
    
    private VideoCapture cap;
    private Mat matFrame = new Mat();
    private JFrame frame;
    private JLabel imgCaptureLabel;
    private JLabel imgDetectionLabel;
    // private CaptureTask captureTask;
    
    public ThresholdInRange() {
        int cameraDevice = 0;
        
        // if (args.length > 0) {   
        //     cameraDevice = Integer.parseInt(args[0]);
        // }

        //cap = new VideoCapture(cameraDevice);
        

        // if (!cap.isOpened()) {
        //     System.err.println("Cannot open camera: " + cameraDevice);
        //     System.exit(0);
        // }
        // if (!cap.read(matFrame)) {
        //     System.err.println("Cannot read camera stream.");
        //     System.exit(0);
        // }

        // Create and set up the window.
        // frame = new JFrame(WINDOW_NAME);s
        // frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        // frame.addWindowListener(new WindowAdapter() {
        //     @Override
        //     public void windowClosing(WindowEvent windowEvent) {
        //         // captureTask.cancel(true);
        //     }
        // });
        // // Set up the content pane.
        // Image img = HighGui.toBufferedImage(matFrame);
        // addComponentsToPane(frame.getContentPane(), img);
        // // Use the content pane's default BorderLayout. No need for
        // // setLayout(new BorderLayout());
        // // Display the window.
        
        // frame.pack();
        // frame.setVisible(true);
        // captureTask = new CaptureTask();
        // captureTask.execute();
    }
    
    private class CaptureTask extends SwingWorker<Void, Mat> {
        @Override
        protected Void doInBackground() {
            Mat matFrame = new Mat();
            while (!isCancelled()) {
                // if (!cap.read(matFrame)) {
                //     break;
                // }
                // publish(matFrame.clone());
            }
            return null;
        }

        @Override
        protected void process(List<Mat> frames) {
        	
            Mat frame = frames.get(frames.size() - 1);
            Mat frameHSV = new Mat();
            Imgproc.cvtColor(frame, frameHSV, Imgproc.COLOR_BGR2HSV);
            Mat thresh = new Mat();
            
            //red color:
//            Core.inRange(frameHSV, new Scalar(0, 130, 130),
//                    new Scalar(180, 240, 255), thresh);
            
            //blue color:
            Core.inRange(frameHSV, new Scalar(95, 100, 90),
                    new Scalar(110, 255, 255), thresh);

            Core.split(thresh, frames);
            Mat gray = frames.get(0);

         //Default:
//            Core.inRange(frameHSV, new Scalar(sliderLowH.getValue(), sliderLowS.getValue0(), sliderLowV.getValue()),
//                    new Scalar(sliderHighH.getValue(), sliderHighS.getValue(), sliderHighV.getValue()), thresh);

              Imgproc.putText(frame, ".", new Point(screenCenterX, screenCenterY), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 3);	
              
              if (varForTimer  == 0) {
            	  varForTimer = 1;
            	  Imgproc.medianBlur(gray, gray, 5);
		          Mat circles = new Mat();    	        	  
		                  
		          Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 2.0,
		        		  			2*(double)gray.rows(), // change this value to detect circles with different distances to each other
		        		  			50.0, 30.0, 0, 0); // change the last two parameters
		                          // (min_radius & max_radius) to detect larger circles - need to change min radius to normal values
		          for (int x = 0; x < circles.cols(); x++) {
		        	  double[] c = circles.get(0, x);
		              Point center = new Point(Math.round(c[0]), Math.round(c[1]));
		                      
		              int cX = (int) Math.round(c[0]/5 - 1)*5; //coordinatesX and coordinatesY
		              int cY = (int) Math.round(c[1]/5 - 1)*5;
		                      
		              String coordinateXY = cX + "," + cY;
		                     
		                      // circle center
		              Imgproc.circle(frame, center, 1, new Scalar(0,255,100), 3, 8, 0);
		                      // circle outline
		              int radius = (int) Math.round(c[2]);
		              Imgproc.circle(frame, center, radius, new Scalar(255,0,255), 3, 8, 0);
		                      
		              Imgproc.putText(frame, coordinateXY, new Point(cX, cY), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 111), 2);	
                    
//                      Imgproc.putText(frame, text, coordinates, fontType, fontSize, color, thickness)
		              
		                      
		                    //distance from camera to ball
                      distanceCameraToBall = Math.round(focalLength*ballRadius/radius) - depth;
                      
                      		//distance from robot to ball                      
                      ballDistance = (double) Math.round(distanceCameraToBall*Math.sin(Math.toRadians(cameraAngle))/2)*2- robotDepth;
                      
                      String Dsize = "Distance: " + ballDistance + "cm";
                      int kat1 = cX-screenCenterX;
                      int kat2 = cY-screenCenterY;                      
                    
                      double ballAngleX = (double) Math.round(Math.toDegrees(Math.atan(ballRadius*kat1/(radius*ballDistance)))/5)*5;
                      double ballAngleY = (double) Math.round(Math.toDegrees(Math.atan(ballRadius*kat2/(radius*ballDistance)))/5)*5;

                      
//                      String string = ballDistance + " ";
//                      Imgproc.putText(frame, string, new Point(20, 300), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 100, 0), 4);
                      
//						new Point(x, y) 
                      //showing angles and distance on the screen
                      String stringBallAngleX = ballAngleX + " X";
                      String stringBallAngleY =  ballAngleY + " Y "; 
                      Imgproc.putText(frame, Dsize, new Point(50, 50), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 2);	
                      Imgproc.putText(frame, stringBallAngleX, new Point(20, 100), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 4);	
                      Imgproc.putText(frame, stringBallAngleY, new Point(20, 150), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 4);
                  }
                
	              update(frame, thresh);
		                  
                  Timer timer1 = new Timer();
                  timer1.schedule(new TimerTask() {
                      public void run() {
                      	varForTimer = 0;
                      }
                  }, 50);   
              }
        }
    }
   
    public double getDistance(){
        return ballDistance;

    }
    private void addComponentsToPane(Container pane, Image img) {
        if (!(pane.getLayout() instanceof BorderLayout)) {
            pane.add(new JLabel("Container doesn't use BorderLayout!"));
            return;
        }

        JPanel framePanel = new JPanel();
        imgCaptureLabel = new JLabel(new ImageIcon(img));
        framePanel.add(imgCaptureLabel);
        imgDetectionLabel = new JLabel(new ImageIcon(img));
        framePanel.add(imgDetectionLabel);
        pane.add(framePanel, BorderLayout.CENTER);
        
    }
    private void update(Mat imgCapture, Mat imgThresh) {
        imgCaptureLabel.setIcon(new ImageIcon(HighGui.toBufferedImage(imgCapture)));
        imgDetectionLabel.setIcon(new ImageIcon(HighGui.toBufferedImage(imgThresh)));
        frame.repaint();
    }
        
}
 