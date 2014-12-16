using AR.Drone.Avionics.Tools.Time;
using AR.Drone.Data.Navigation;
using AR.Drone.Infrastructure;
using AR.Drone.Client;
using AR.Drone.Avionics.Objectives;
namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    public partial class KinectWindow : Window, INotifyPropertyChanged
    {
        //  ARDrone client - this enables u to connect to the ARDrone and use Roslan's
        // C# flight controls
        DroneClient droneClient = null;
        
        // booleans for ardrone behavior ( booleans are used so that mutiple commands can be executed in on  message)
         
        bool isFlying = false;
        bool isRightThumbUp = false;
        bool isRightThumbOut = false;
        bool isRightHandRight = false;
        bool isRightHandLeft = false;
        bool isLeftHandRight = false;
        bool isLeftHandLeft = false;
        bool isLeftThumbUp = false;
        bool isLeftThumbOut = false;
        bool isLeftHandUp = false;

        // offsets for checking hand motions
        
        private const double ThumbUp = .05;
        private const double ThumbRightOffset = -0.04;
        private const double LeftThumbOutOffset = .04;
        private const double normalOffset = -.1;
        private const double RightHandOffset = .1;
        private const double RightHandLeftOffset = -.1;


        //Variables used for drawing on the kinect window
        private const double HandSize = 30;
        private const double JointThickness = 3;
        private const double ClipBoundsThickness = 10;
        private const float InferredZPositionClamp = 0.1f;
        private int displayWidth;
        private int displayHeight;
        private List<Pen> bodyColors;
        private string statusText = null;
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));      
        private readonly Brush inferredJointBrush = Brushes.Yellow;       
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        private KinectSensor kinectSensor = null;
        private CoordinateMapper coordinateMapper = null;
        private BodyFrameReader bodyFrameReader = null;
        private Body[] bodies = null;
        // joints have have coordinance that can be used to find where body parts
        // are located in relation to other body parts
        private List<Tuple<JointType, JointType>> bones;

        public KinectWindow()
        {
            // connect to the Kinect and ARDrone
            this.kinectSensor = KinectSensor.GetDefault();
            Console.WriteLine("KINECT REACHED");
            // standard ip for the ARDrone 2.0 is 192.168.1.1
            droneClient = new DroneClient("192.168.1.1");
            // initiates connection
            droneClient.Start();
            Console.WriteLine("ardrone is connected ="+droneClient.IsConnected);
            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            // get the depth (display) extents
            // this is used to find the max x and y coordinates
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;
            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            // a bone defined as a line between two joints
            /*
                AnkleLeft 14   Left ankle 
                AnkleRight 18   Right ankle 
                ElbowLeft 5   Left elbow 
                ElbowRight 9   Right elbow 
                FootLeft 15   Left foot 
                FootRight 19   Right foot 
                HandLeft 7   Left hand 
                HandRight 11   Right hand 
                HandTipLeft 21   Tip of the left hand 
                HandTipRight 23   Tip of the right hand 
                Head 3   Head 
                HipLeft 12   Left hip 
                HipRight 16   Right hip 
                KneeLeft 13   Left knee 
                KneeRight 17   Right knee 
                Neck 2   Neck 
                ShoulderLeft 4   Left shoulder 
                ShoulderRight 8   Right shoulder 
                SpineBase 0   Base of the spine 
                SpineMid 1   Middle of the spine 
                SpineShoulder 20   Spine at the shoulder 
                ThumbLeft 22   Left thumb 
                ThumbRight 24   Right thumb 
                WristLeft 6   Left wrist 
                WristRight 10   Right wrist 
                */
            this.bones = new List<Tuple<JointType, JointType>>();
            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));
            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));
            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));
            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));
            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));
            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();
            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));
            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;
            // open the sensor
            this.kinectSensor.Open();
            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();
            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);
            // use the window object as the view model in this simple example
            this.DataContext = this;
            // initialize the components (controls) of the window
            this.InitializeComponent();
        }
        public event PropertyChangedEventHandler PropertyChanged;
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }
        /// Gets or sets the current status text to display
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }
        /// Execute start up tasks
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }
        /// Execute shutdown tasks
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }
        /// Handles the body frame data arriving from the sensor
        // this is where most of the logic is done ****
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);
                            // ** Important bools for flight controls **
                            //-----------------------------------------------------------------------------------------------
                            // 1. checks if the right arm is in normal position ( take off )
                            // 3. bool isRightThumbUp checks if the right thumb is up   ( bank right )
                            // 4. bool isRightThumbOut checks if the right thumb is out to the side ( bank left )
                            // 5. bool isRightHandRight checks if right hand is to the right of the elbow  ( rotate right)
                            // 6. bool isRightHandLeft checks if the right hand is to the left of the elbow  ( rotate left )
                            // 7. bool isLeftThumbOut checks if the left thumb is out to the right   (go backwards)
                            // 8. bool isLeftThumbUP checks if the left thumb is up (go forward)
                            // 9. bool isLeftHandRight checks if the left hand is to the right of the left elbow (go higher)
                            //10. bool isLeftHandLeft if the left hand is to the left of the left elbow (go lower)
                            //11. bool isLeftHandUp checks if the left arm is up (initiate left hand controls)
                            //------------------------------------------------------------------------------------------------

                            //checks if right hand is raised
                            if (isAbove(body, JointType.HandRight, JointType.ElbowRight, normalOffset))
                            {
                                if (droneClient != null && this.isFlying != true)
                                {
                                    // ARDrone will take of if it is not flying already
                                    droneClient.Takeoff();
                                    Console.WriteLine("TAKEOFF...");
                                    // 7 seconds is given for the ARDrone to take off
                                    System.Threading.Thread.Sleep(7000);
                                    // isFlying is set true so the ARDrone knows to land if the right hand is down
                                    isFlying = true;
                                    return;
                                }
                                //Every frame that the Kinect processes it will update the flight control bools
                                isLeftHandUp = isAbove(body, JointType.HandLeft, JointType.ElbowLeft, normalOffset);
                                isRightThumbUp = isAbove(body, JointType.ThumbRight, JointType.HandRight, ThumbUp);
                                isRightThumbOut = isLeft(body, JointType.ThumbRight, JointType.HandRight, ThumbRightOffset);
                                isRightHandRight = isRight(body, JointType.HandRight,JointType.ElbowRight,RightHandOffset);
                                isRightHandLeft = isLeft(body, JointType.HandRight, JointType.ElbowRight, RightHandLeftOffset);
                                isLeftHandRight = isRight(body, JointType.HandLeft,JointType.ElbowLeft,RightHandOffset);
                                isLeftHandLeft = isLeft(body, JointType.HandLeft, JointType.ElbowLeft, RightHandLeftOffset);
                                isLeftThumbUp = isAbove(body, JointType.ThumbLeft, JointType.HandLeft, ThumbUp);
                                isLeftThumbOut = isRight(body, JointType.ThumbLeft, JointType.HandLeft, LeftThumbOutOffset);
                                updateFlight();
                                //  DrawHand is used for testing and can be updated to draw to the screen based on the bools
                                this.DrawHand(isRightThumbUp, body.HandRightState, jointPoints[JointType.HandRight], dc);
                            }
                            else
                            {
                                // if right hand is down and the drone is flying then land the ARDrone
                                droneClient.Land();
                                isFlying = false;
                            }                            
                        }
                    }
                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }
        //----------------------------------------------------------------------------------------------------//
        //                                      Flight controls
        // roll = left/right (banking)
        // pitch = forwards/backwards
        // gaz = altitude (gaz does not work too well and it may take some playing with to perfect)
        // yaw = rotation
        //----------------------------------------------------------------------------------------------------//
        private bool updateFlight()
        {
            float roll = 0;
            float pitch = 0;
            float yaw = 0;
            float gaz = 0;
            // this ensures that the drone will hover if no new commands are sent
            if((isLeftHandUp && !isLeftThumbUp && !isLeftThumbOut && !isRightThumbOut && !isRightThumbUp && !isRightHandRight && !isRightHandLeft) || ( !isLeftHandUp && !isRightThumbOut && !isRightThumbUp && !isRightHandRight && !isRightHandLeft))
            {
                Console.WriteLine("hover");
                droneClient.Progress(AR.Drone.Client.Command.FlightMode.Progressive, 0, 0, 0, 0);
                return true;

            }
            // sends the ARDrone to the right at the speed of .1f
            if(isRightThumbUp)
            {
                Console.WriteLine("go right");
                roll = 0.1f;
                isRightThumbUp = false;
               
            }
            // sends the ARDrone to the left at the speed of .1f
            if(isRightThumbOut)
            {
                Console.WriteLine("go left");
                roll = -0.1f;
                isRightThumbOut = false;
   
            }
            // sends the ARDrone upwards at the speed of .1f
            if (isLeftHandRight && isLeftHandUp)
            {
                Console.WriteLine("raise altitude");
                //raise altitude
                //gaz = .1f;
                isRightHandRight = false;
      
            }
            // sends the ARDrone downwards at the speed of .1f
            else if (isLeftHandLeft && isLeftHandUp)
            {
                Console.WriteLine("lower altitude");
                //TODO rotate counter clockwise
                //gaz = -.1f;
                isRightHandLeft = false;
                
            }
            //rotates the ARDrone counter clockwise at .1f
            if (isRightHandRight)
            {
                Console.WriteLine("rotate");
                yaw = .1f;
                isRightHandRight = false;

            }
            //rotates the ARDrone clockwise at .1f
            else if (isRightHandLeft)
            {
                Console.WriteLine("rotate");
                //TODO rotate counter clockwise
                yaw = -.1f;
                isRightHandLeft = false;

            }
            //Sends the ARDrone Forwards
            if (isLeftThumbUp && isLeftHandUp)
            {
                Console.WriteLine("go forward");
                pitch = -.1f;
                isLeftThumbUp = false;
                
            }
            //Sends the ARDrone Backwards
            else if (isLeftThumbOut && isLeftHandUp)
            {
                Console.WriteLine("go backwards");
                pitch = 0.1f;
                isLeftThumbOut = false;
               
            }
            //Publishes all of the flight commands at once so that mutiple commands can be handled at one time
            droneClient.Progress(AR.Drone.Client.Command.FlightMode.Progressive, roll, pitch, yaw, gaz);
            return true;
        }

        // These functions are used to determine where body parts are in relation to others
        // These functions are useful in for the flight control bools

        // detects if hand is above head
        //Detects if the first argument is above the second
        private bool isAbove(Body body,JointType jointAbove, JointType jointBelow)
        {
            var above = body.Joints[jointAbove];
            var below = body.Joints[jointBelow];
            bool isDetected = above.Position.Y > below.Position.Y;
            return isDetected;
        }
        // this overload checks to see if the jointAbove is greater than  jointBelow + a offset in meters?  
        // -- this is useful for thing like checking if the thumb is actually up                            
        private bool isAbove(Body body, JointType jointAbove, JointType jointBelow,double offset)
        {
            var above = body.Joints[jointAbove];
            var below = body.Joints[jointBelow];
            bool isDetected = above.Position.Y > below.Position.Y + offset;
            return isDetected;
        }
        // This method checks if the JointOne is to left of jointTwo plus an offset                                                                                                  
        private bool isLeft(Body body, JointType jointOne, JointType jointTwo, double offset)
        {
            var above = body.Joints[jointOne];
            var below = body.Joints[jointTwo];
            bool isDetected = above.Position.X < below.Position.X + offset;
            return isDetected;
        }
        // This method checks if the JointOne is to right of jointTwo plus an offset                                                                                                  
        private bool isRight(Body body, JointType jointOne, JointType jointTwo, double offset)
        {
            var one = body.Joints[jointOne];
            var two = body.Joints[jointTwo];
            bool isDetected = one.Position.X > two.Position.X + offset;
            return isDetected;
        }



        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        //
        // This method can be used for testing to draw circles based on the boolean isDetected
        //
        private void DrawHand(bool isDetected, HandState handState, Point handPosition, DrawingContext drawingContext)
        {
           // use handstate to check if the hand is open or closed and stuff like that
            switch (isDetected)
            {
                case false:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;
                case true:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
