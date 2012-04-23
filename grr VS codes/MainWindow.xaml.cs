/////////////////////////////////////////////////////////////////////////
//
// This module contains code to do Kinect NUI initialization and
// processing and also to display NUI streams on screen.
//
// Copyright © Microsoft Corporation.  All rights reserved.  
// This code is licensed under the terms of the 
// Microsoft Kinect for Windows SDK (Beta) from Microsoft Research 
// License Agreement: http://research.microsoft.com/KinectSDK-ToU
//
/////////////////////////////////////////////////////////////////////////
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Research.Kinect.Nui;
using System.IO.Ports;

namespace SkeletalViewer
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Constructors
        public MainWindow()
        {
            InitializeComponent();
            // Initialize the serial port and open it for communication.
            this.serialPort = new SerialPort("COM4",9600, Parity.None, 8, StopBits.One);
            try
            {
                this.serialPort.Open();
            }
            catch (Exception)
            {
                MessageBox.Show("Error opening serial port for communication.", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }

          // this.state = false;
            //this.clapStarted = false;
        }
        #endregion

        #region Variables for initialization of Kinect
        Runtime nui;
        int totalFrames = 0;
        int lastFrames = 0;
        DateTime lastTime = DateTime.MaxValue;

        // We want to control how depth data gets converted into false-color data
        // for more intuitive visualization, so we keep 32-bit color frame buffer versions of
        // these, to be updated whenever we receive and process a 16-bit frame.
        const int RED_IDX = 2;
        const int GREEN_IDX = 1;
        const int BLUE_IDX = 0;
        byte[] depthFrame32 = new byte[320 * 240 * 4];

        Dictionary<JointID, Brush> jointColors = new Dictionary<JointID, Brush>() { 
            {JointID.HipCenter, new SolidColorBrush(Color.FromRgb(169, 176, 155))},
            {JointID.Spine, new SolidColorBrush(Color.FromRgb(169, 176, 155))},
            {JointID.ShoulderCenter, new SolidColorBrush(Color.FromRgb(168, 230, 29))},
            {JointID.Head, new SolidColorBrush(Color.FromRgb(200, 0,   0))},
            {JointID.ShoulderLeft, new SolidColorBrush(Color.FromRgb(79,  84,  33))},
            {JointID.ElbowLeft, new SolidColorBrush(Color.FromRgb(84,  33,  42))},
            {JointID.WristLeft, new SolidColorBrush(Color.FromRgb(255, 126, 0))},
            {JointID.HandLeft, new SolidColorBrush(Color.FromRgb(215,  86, 0))},
            {JointID.ShoulderRight, new SolidColorBrush(Color.FromRgb(33,  79,  84))},
            {JointID.ElbowRight, new SolidColorBrush(Color.FromRgb(33,  33,  84))},
            {JointID.WristRight, new SolidColorBrush(Color.FromRgb(77,  109, 243))},
            {JointID.HandRight, new SolidColorBrush(Color.FromRgb(37,   69, 243))},
            {JointID.HipLeft, new SolidColorBrush(Color.FromRgb(77,  109, 243))},
            {JointID.KneeLeft, new SolidColorBrush(Color.FromRgb(69,  33,  84))},
            {JointID.AnkleLeft, new SolidColorBrush(Color.FromRgb(229, 170, 122))},
            {JointID.FootLeft, new SolidColorBrush(Color.FromRgb(255, 126, 0))},
            {JointID.HipRight, new SolidColorBrush(Color.FromRgb(181, 165, 213))},
            {JointID.KneeRight, new SolidColorBrush(Color.FromRgb(71, 222,  76))},
            {JointID.AnkleRight, new SolidColorBrush(Color.FromRgb(245, 228, 156))},
            {JointID.FootRight, new SolidColorBrush(Color.FromRgb(77,  109, 243))}
        };
        #endregion

        #region Our Variables
        /// <summary>
        /// Variable to track if the robot should be moving or should be in a stop state.
        /// </summary>
        private bool state;

        /// <summary>
        /// Flag to detect if the user is in the middle of a clap.
        /// </summary>
        private bool clapStarted;

        // Variables for serial port communication.
        SerialPort serialPort;

        // Variables to trck the position of various co-ordinates.
        double r_wrist_y = 0;
        double l_wrist_y = 0;
        double r_wrist_x = 0;
        double l_wrist_x = 0;
        double r_wrist_z = 0;
        double l_wrist_z = 0;
        double l_ankle_y = 0;
        double r_ankle_y = 0;
        double l_ankle_x = 0;
        double r_ankle_x = 0;


        double head_y = 0;
        double head_x = 0;
        double hip_y = 0;
        double hip_x = 0;

        private double centreX;
        private double centreY;

        private double r_shoulder_x = 0;
        private double r_shoulder_y = 0;
        private double l_shoulder_x = 0;
        private double l_shoulder_y = 0;
        private double r_hip_x = 0;
        private double r_hip_y = 0;
        private double l_hip_x = 0;
        private double l_hip_y = 0;
        

        #endregion

        private void Window_Loaded(object sender, EventArgs e)
        {
            nui = new Runtime();

            try
            {
                nui.Initialize(RuntimeOptions.UseDepthAndPlayerIndex | RuntimeOptions.UseSkeletalTracking | RuntimeOptions.UseColor);
            }
            catch (InvalidOperationException)
            {
                System.Windows.MessageBox.Show("Runtime initialization failed. Please make sure Kinect device is plugged in.");
                return;
            }


            try
            {
                nui.VideoStream.Open(ImageStreamType.Video, 2, ImageResolution.Resolution640x480, ImageType.Color);
                nui.DepthStream.Open(ImageStreamType.Depth, 2, ImageResolution.Resolution320x240, ImageType.DepthAndPlayerIndex);
            }
            catch (InvalidOperationException)
            {
                System.Windows.MessageBox.Show("Failed to open stream. Please make sure to specify a supported image type and resolution.");
                return;
            }

            lastTime = DateTime.Now;

            nui.DepthFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_DepthFrameReady);
            nui.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
            nui.VideoFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_ColorFrameReady);
        }

        // Converts a 16-bit grayscale depth frame which includes player indexes into a 32-bit frame
        // that displays different players in different colors
        byte[] convertDepthFrame(byte[] depthFrame16)
        {
            for (int i16 = 0, i32 = 0; i16 < depthFrame16.Length && i32 < depthFrame32.Length; i16 += 2, i32 += 4)
            {
                int player = depthFrame16[i16] & 0x07;
                int realDepth = (depthFrame16[i16 + 1] << 5) | (depthFrame16[i16] >> 3);
                // transform 13-bit depth information into an 8-bit intensity appropriate
                // for display (we disregard information in most significant bit)
                byte intensity = (byte)(255 - (255 * realDepth / 0x0fff));

                depthFrame32[i32 + RED_IDX] = 0;
                depthFrame32[i32 + GREEN_IDX] = 0;
                depthFrame32[i32 + BLUE_IDX] = 0;

                // choose different display colors based on player
                switch (player)
                {
                    case 0:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity / 2);
                        break;
                    case 1:
                        depthFrame32[i32 + RED_IDX] = intensity;
                        break;
                    case 2:
                        depthFrame32[i32 + GREEN_IDX] = intensity;
                        break;
                    case 3:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity / 4);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity);
                        break;
                    case 4:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity / 4);
                        break;
                    case 5:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity / 4);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity);
                        break;
                    case 6:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity);
                        break;
                    case 7:
                        depthFrame32[i32 + RED_IDX] = (byte)(255 - intensity);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(255 - intensity);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(255 - intensity);
                        break;
                }
            }
            return depthFrame32;
        }

        void nui_DepthFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            PlanarImage Image = e.ImageFrame.Image;
            byte[] convertedDepthFrame = convertDepthFrame(Image.Bits);

            depth.Source = BitmapSource.Create(
                Image.Width, Image.Height, 96, 96, PixelFormats.Bgr32, null, convertedDepthFrame, Image.Width * 4);

            ++totalFrames;

            DateTime cur = DateTime.Now;
            if (cur.Subtract(lastTime) > TimeSpan.FromSeconds(1))
            {
                int frameDiff = totalFrames - lastFrames;
                lastFrames = totalFrames;
                lastTime = cur;
                frameRate.Text = frameDiff.ToString() + " fps by team 16";
            }

            #region Standard movement
            //TEAM 16_2012
            //string dbg_msg = "wr: " + r_wrist_y + " hd: " + head_y;
            string dbg_msg = "ak:" + r_ankle_y + "hp:" + hip_y;
            //gexturePrint.Text = dbg_msg;

         //   if ((r_wrist_y < l_wrist_y + 40 && r_wrist_y > l_wrist_y - 40) && (r_wrist_x < l_wrist_x + 40 && r_wrist_x > l_wrist_x - 40))
            {
                this.clapStarted = true;
            }
         //   else if (this.clapStarted)
            {
                this.clapStarted = false;
                this.state = !this.state;
            }

            if (r_ankle_y > l_ankle_y)
            {
                gexturePrint.Text = "Command : stop";
                this.serialPort.Write("5");
 
            }
            else if (r_wrist_y < head_y)
            {
               gexturePrint.Text = "Command : Right";
                this.serialPort.Write("6");
            }
            else if (l_wrist_y < head_y)
            {
                gexturePrint.Text = "Command : Left";
                this.serialPort.Write("4");
            }// code start
            else if (r_wrist_y < l_wrist_y)// if ((r_wrist_y < head_y) && (l_wrist_y < head_y))
            {
              gexturePrint.Text = "Command : Backward";
                this.serialPort.Write("2");
            }
            else if(r_wrist_y > l_wrist_y)
            {
                gexturePrint.Text = "Command : Forward";
                this.serialPort.Write("8");
            }// code end
            else if (l_ankle_y > r_ankle_y)
            {
                gexturePrint.Text = "Command : Buzzer on ";
                this.serialPort.Write("7");

                gexturePrint.Text = "Command : Buzzer off ";
                this.serialPort.Write("9");

            }
            //else if ((r_wrist_y > head_y) && (l_wrist_y > head_y)&&(r_ankle_y > l_ankle_y))
           {
                //if (this.state)
               // {
               //  gexturePrint.Text = "Command : Stop";
               //  this.serialPort.Write("5");
               // }
                //if (this.state)
                //{
                //    gexturePrint.Text = "Command : Forward";
                //    this.serialPort.Write("8");
                //}
                //else
                //{
                //    gexturePrint.Text = "Command : Stop";
                //    this.serialPort.Write("5");

                //}
            }

            
            #endregion

            #region Flag Semaphores
            // Detecting various semaphores.
            double left_hand_slope = this.calculateSlope(this.l_shoulder_x, this.l_shoulder_y, this.l_wrist_x, this.l_wrist_y);
            double right_hand_slope = this.calculateSlope(this.r_shoulder_x, this.r_shoulder_y, this.r_wrist_x, this.r_wrist_y);
            double left_leg_slope = this.calculateSlope(this.l_hip_x, this.l_hip_y, this.l_ankle_x, this.l_ankle_y);
            double right_leg_slope = this.calculateSlope(this.r_hip_x, this.r_hip_y, this.r_ankle_x, this.r_ankle_y);
            this.semaphore.Text = "L: " + left_hand_slope + " R: " + right_hand_slope;
           // this.semaphore.Text = "Le: " + left_leg_slope + "Re: " + right_leg_slope;
            #endregion
        }

        private double calculateSlope(double x1, double y1, double x2, double y2)
        {
            double slope = 0;

            if (Math.Abs(x2 - x1) != 0)
            {
                slope = Math.Atan2((y2 - y1), (x2 - x1));
            }
            else
            {
                slope = (y1 < y2) ? 0 : Math.PI;
            }

            return Math.Round((slope * 180) / Math.PI, 0);
        }

        private Point getDisplayPosition(Joint joint)
        {
            float depthX, depthY;
            nui.SkeletonEngine.SkeletonToDepthImage(joint.Position, out depthX, out depthY);
            depthX = depthX * 320; //convert to 320, 240 space
            depthY = depthY * 240; //convert to 320, 240 space
            int colorX, colorY;
            ImageViewArea iv = new ImageViewArea();
            // only ImageResolution.Resolution640x480 is supported at this point
            nui.NuiCamera.GetColorPixelCoordinatesFromDepthPixel(ImageResolution.Resolution640x480, iv, (int)depthX, (int)depthY, (short)0, out colorX, out colorY);

            // map back to skeleton.Width & skeleton.Height
            return new Point((int)(skeleton.Width * colorX / 640.0), (int)(skeleton.Height * colorY / 480));
        }

        Polyline getBodySegment(Microsoft.Research.Kinect.Nui.JointsCollection joints, Brush brush, params JointID[] ids)
        {
            PointCollection points = new PointCollection(ids.Length);
            for (int i = 0; i < ids.Length; ++i)
            {
                points.Add(getDisplayPosition(joints[ids[i]]));
            }

            Polyline polyline = new Polyline();
            polyline.Points = points;
            polyline.Stroke = brush;
            polyline.StrokeThickness = 5;
            return polyline;
        }

        void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            SkeletonFrame skeletonFrame = e.SkeletonFrame;
            int iSkeleton = 0;
            Brush[] brushes = new Brush[6];
            brushes[0] = new SolidColorBrush(Color.FromRgb(255, 0, 0));
            brushes[1] = new SolidColorBrush(Color.FromRgb(0, 255, 0));
            brushes[2] = new SolidColorBrush(Color.FromRgb(64, 255, 255));
            brushes[3] = new SolidColorBrush(Color.FromRgb(255, 255, 64));
            brushes[4] = new SolidColorBrush(Color.FromRgb(255, 64, 255));
            brushes[5] = new SolidColorBrush(Color.FromRgb(128, 128, 255));

            skeleton.Children.Clear();
            foreach (SkeletonData data in skeletonFrame.Skeletons)
            {
                if (SkeletonTrackingState.Tracked == data.TrackingState)
                {
                    // Draw bones
                    Brush brush = brushes[iSkeleton % brushes.Length];
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.Spine, JointID.ShoulderCenter, JointID.Head));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.ShoulderCenter, JointID.ShoulderLeft, JointID.ElbowLeft, JointID.WristLeft, JointID.HandLeft));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.ShoulderCenter, JointID.ShoulderRight, JointID.ElbowRight, JointID.WristRight, JointID.HandRight));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.HipLeft, JointID.KneeLeft, JointID.AnkleLeft, JointID.FootLeft));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.HipRight, JointID.KneeRight, JointID.AnkleRight, JointID.FootRight));

                    // Draw joints
                    //motion = 0;
                    foreach (Joint joint in data.Joints)
                    {
                        Point jointPos = getDisplayPosition(joint);
                        Line jointLine = new Line();
                        jointLine.X1 = jointPos.X - 3;
                        jointLine.X2 = jointLine.X1 + 6;
                        jointLine.Y1 = jointLine.Y2 = jointPos.Y;
                        jointLine.Stroke = jointColors[joint.ID];
                        jointLine.StrokeThickness = 6;
                        skeleton.Children.Add(jointLine);
                        if (joint.ID == JointID.WristRight)
                        {
                            r_wrist_y = jointPos.Y;
                            r_wrist_x = jointPos.X;
                        }
                        if (joint.ID == JointID.WristLeft)
                        {
                            l_wrist_y = jointPos.Y;
                            l_wrist_x = jointPos.X;
                        }
                        if (joint.ID == JointID.Head)
                        {
                            head_y = jointPos.Y;
                        }

                        if (joint.ID == JointID.Spine)
                        {
                            this.centreX = joint.Position.X;
                            this.centreY = joint.Position.Y;
                        }

                        if (joint.ID == JointID.ElbowRight)
                        {
                            this.r_shoulder_x = joint.Position.X;
                            this.r_shoulder_y = joint.Position.Y;
                        }

                        if (joint.ID == JointID.ElbowLeft)
                        {
                            this.l_shoulder_x = joint.Position.X;
                            this.l_shoulder_y = joint.Position.Y;
                        }
                        if (joint.ID == JointID.AnkleLeft)
                        {
                            this.l_ankle_y = jointPos.Y;
                            this.l_ankle_x = jointPos.X;
                        }
                        if (joint.ID == JointID.AnkleRight)
                        {
                            this.r_ankle_y = jointPos.Y;
                            this.r_ankle_x = jointPos.X;
                        }
                        if (joint.ID == JointID.KneeRight)
                        {
                            this.r_hip_x = joint.Position.X;
                            this.r_hip_y = joint.Position.Y;
                        }
                        if (joint.ID == JointID.KneeLeft)
                        {
                            this.l_hip_x = joint.Position.X;
                            this.l_hip_y = joint.Position.Y;
                        }
                    }
                }
                iSkeleton++;
            } // for each skeleton
        }

        void nui_ColorFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            // 32-bit per pixel, RGBA image
            PlanarImage Image = e.ImageFrame.Image;
            video.Source = BitmapSource.Create(
                Image.Width, Image.Height, 96, 96, PixelFormats.Bgr32, null, Image.Bits, Image.Width * Image.BytesPerPixel);
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            nui.Uninitialize();
            // Close the serial port.
            try
            {
                this.serialPort.Close();
            }
            catch (InvalidOperationException)
            {
                MessageBox.Show("An error occurred while closing the port.", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
            Environment.Exit(0);
        }
    }
}
