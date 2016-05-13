//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Threading;
using Microsoft.Kinect;
using Accord.Imaging;
using Accord.Imaging.Filters;
using Accord.Math.Geometry;
using Accord.Vision.Tracking;
using AForge;
using AForge.Imaging;
using AForge.Imaging.Filters;
using AForge.Math.Geometry;
using AForge.Video;
using AForge.Video.DirectShow;
using AForge.Video.VFW;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace Microsoft.Samples.Kinect.ColorBasics
{
    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for color frames
        /// </summary>
        private ColorFrameReader colorFrameReader = null;

        /// <summary>
        /// Reader for multi frames
        /// </summary>
        private MultiSourceFrameReader multiReader = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;

        private Bitmap image = null;

        Bitmap searchArea;
        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private HslBlobTracker tracker;

        BorderFollowing bf = new BorderFollowing(10);

        KCurvature kcurv = new KCurvature(30, new DoubleRange(0, 45));

        GaussianBlur blur = new GaussianBlur(1.1);

        PointsMarker cmarker = new PointsMarker(System.Drawing.Color.White, 1);

        FrameDescription colorFrameDescription = null;
        FrameDescription depthFrameDescription = null;

        IntPoint center = new IntPoint(0, 0);

        private ushort[] depthArray;

        private float _x = 0;
        private float _y = 0;
        private float _depth = 0;
        
        //creates calibration variables as well as he abdomenMap list
        private float initialDepth = 0;
        private float calibratedDepth = 0;
        private float calibratedX = 0;
        private float calibratedY = 0;
        private double length = 0.32;
        List<Point3D> abdomenMap = new List<Point3D>();

        private PointsVisual3D pointVisual = new PointsVisual3D { Color = Colors.Red, Size = 5 };
        private Point3DCollection trackedPoint;
        private PointsVisual3D cloudVisual = new PointsVisual3D { Color = Colors.Blue, Size = 2 };
        private Point3DCollection pointCloud;
        private CameraSpacePoint[] csp;

        private bool mapCloud = false;

        private System.Drawing.Size searchSize = new System.Drawing.Size(175, 175);
        private Rectangle searchRect;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();
    
            multiReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);
            
            this.multiReader.MultiSourceFrameArrived += multiReader_MultiSourceFrameArrived;

            // open the reader for the color frames
//            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();
            
            // wire handler for frame arrival
//            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // create the bitmap to display
            //this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            //this.colorBitmap = new WriteableBitmap(trackingWidth, trackingHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

            center.X = colorFrameDescription.Width / 2;
            center.Y = colorFrameDescription.Height / 2;

            this.image = new Bitmap(colorFrameDescription.Width, colorFrameDescription.Height, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
            searchArea = new Bitmap(searchSize.Width, searchSize.Height, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            tracker = new HslBlobTracker();
            tracker.Filter.Hue = new IntRange(60, 145);
            tracker.Filter.Saturation = new Range(0.353f, 1.0f);
            tracker.Filter.Luminance = new Range(0.125f, 1.0f);
            tracker.MinHeight = 15;
            tracker.MinWidth = 15;
            tracker.MaxHeight = 200;
            tracker.MaxWidth = 200;
            tracker.Extract = true;
            tracker.ComputeOrientation = false;

            depthArray = new ushort[depthFrameDescription.LengthInPixels];
            csp = new CameraSpacePoint[colorFrameDescription.LengthInPixels];
            pointCloud = new Point3DCollection();
            trackedPoint = new Point3DCollection();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        public async void TrackColor()
        {
            BitmapData searchData = searchArea.LockBits(new Rectangle(0, 0, searchArea.Width, searchArea.Height),
                ImageLockMode.ReadWrite, searchArea.PixelFormat);
            UnmanagedImage searchImg = new UnmanagedImage(searchData);
            tracker.ProcessFrame(searchImg);
            if (!tracker.TrackingObject.IsEmpty)
            {
                IntPoint dummyCenter = tracker.TrackingObject.Center;
                center.X += (dummyCenter.X - searchSize.Width / 2);
                center.Y += (dummyCenter.Y - searchSize.Height / 2);
                center.X = (center.X < 0) ? 0 : center.X;
                center.Y = (center.Y < 0) ? 0 : center.Y;
                center.X = (center.X > image.Width - 1) ? image.Width - 1 : center.X;
                center.Y = (center.Y > image.Height - 1) ? image.Height - 1 : center.Y;
            }
            searchArea.UnlockBits(searchData);
        }

        public async void ProcessDepth()
        {

            var pDepthData = GCHandle.Alloc(depthArray, GCHandleType.Pinned);
            var pCameraArray = GCHandle.Alloc(csp, GCHandleType.Pinned);

            csp.Initialize();
            try
            {
                kinectSensor.CoordinateMapper.MapColorFrameToCameraSpaceUsingIntPtr(
                    pDepthData.AddrOfPinnedObject(),
                    (uint)(depthArray.Length * sizeof(ushort)),
                    csp);
            }
            catch
            {

            }

            pCameraArray.Free();
            pDepthData.Free();

//            kinectSensor.CoordinateMapper.MapColorFrameToCameraSpace(depthArray, csp);
            this.Dispatcher.Invoke((Action)(() => {

                if(mapCloud)
                {
                    pointCloud.Clear();
                    HelixImage.Children.Remove(cloudVisual);
                    cloudVisual.Children.Clear();

                    for (int i = 0; i < csp.Length; i++)
                    {
                        if (!float.IsNegativeInfinity(csp[i].X) && !float.IsNegativeInfinity(csp[i].Y) && !float.IsNegativeInfinity(csp[i].Y))
                        {
                            pointCloud.Add(new Point3D(csp[i].X, csp[i].Y, csp[i].Z));
                        }
                    }

                    cloudVisual.Points = pointCloud;
                    HelixImage.Children.Add(cloudVisual);
                    mapCloud = false;
                }

                _x = csp[center.Y * image.Width + center.X].X;
                _y = csp[center.Y * image.Width + center.X].Y;
                _depth = csp[center.Y * image.Width + center.X].Z;

                CenterText = (float.IsNegativeInfinity(_x)) ? "Too close to sensor!" : (_x + ", " + _y + ", " + _depth);
            }));
        }


        //finds distance from kinect camera to the ping pong ball when the tool is at its furthest extended position with the tip at the entrance point
        private void initialDepthCalibration_Click(object sender, EventArgs e)
        {
            initialDepth = _depth;
        }
        //finds the depth that the tool is inserted into the abdomen as well as finding the coordinates of X and Y when the tool is vertical at its current depth
        private void depthCalibration_Click(object sender, EventArgs e)
        {
            calibratedDepth = _depth - initialDepth;
            calibratedX = _x;
            calibratedY = _y;
        }
        //adds a point coordinate to the abdomenMap pointcloud list
        public void addPoint(object sender, EventArgs e)
        {
            // finds initial distances from calibrated coordinates for x, y and z
            double xin = _x - calibratedX;
            double yin = _y - calibratedY;
            double zin = (initialDepth + length) - _depth;
            //creates variables that will store the change in position fromthe calibration point
            double xChange = 0;
            double yChange = 0;
            double zChange = 0;
            // logic sequence to prevent errors from diving by zero
            if ((xin != 0) && (yin != 0))
            {
                // finds angles from x axis (angleXY) and from the xy-plane (angleDL)
                double angleXY = Math.Atan(Math.Abs(yin / xin));
                double angleDL = Math.Asin(Math.Abs(zin / (length - calibratedDepth)));
                //finds change in x, y, and z directions from the calibration point
                xChange = Math.Cos(angleXY) * (Math.Cos(angleDL) * calibratedDepth);
                yChange = Math.Sin(angleXY) * (Math.Cos(angleDL) * calibratedDepth);
                zChange = Math.Sin(angleDL) * calibratedDepth;
            }
            else if ((xin == 0) && (yin != 0))
            {
                xChange = 0;
                yChange = yin;
            }
            else if ((xin != 0) && (yin == 0))
            {
                xChange = xin;
                yChange = 0;
            }
            else if ((xin == 0) && (yin == 0))
            {
                xChange = 0;
                yChange = 0;
                zChange = calibratedDepth;
            }
            //variables created to be part of coordinate added to the point cloud
            double xPoint = 0;
            double yPoint = 0;
            double zPoint = initialDepth + length + zChange;
            //logic sequence to take into account the quadrant in which the new point is to be placed and add/subtracts the (x,y,or z)Change accordingly
            if (_x > calibratedX)
            {
                xPoint = calibratedX - xChange;
            }
            else if(_x < calibratedX)
            {
                xPoint = calibratedX + xChange;
            }
            else if(_x == calibratedX)
            {
                xPoint = calibratedX;
            }
            if (_y > calibratedX)
            {
                yPoint = calibratedY - yChange;
            }
            else if (_y < calibratedY)
            {
               yPoint = calibratedY + yChange;
            }
            else if (_y == calibratedY)
            {
                yPoint = calibratedY;
            }
            //coordinate is finalized need to be able to add it to a list.
            //abdomenMap.Add(xPoint, yPoint, zPoint); find a way to add a 3D point to a list
        }



        void multiReader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame comboFrame = e.FrameReference.AcquireFrame();
            if (comboFrame == null) return;
            
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = comboFrame.ColorFrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        lock (this)
                        {
                            Bitmap image = ImageToBitmap(colorFrame);
                            colorFrame.Dispose();

                            int startX = center.X - searchSize.Width / 2;
                            int startY = center.Y - searchSize.Height / 2;
                            startX = (startX < 0) ? 0 : startX;
                            startY = (startY < 0) ? 0 : startY;
                            startX = ((startX + searchSize.Width) > image.Width) ? (image.Width - searchSize.Width) : startX;
                            startY = ((startY + searchSize.Height) > image.Height) ? (image.Height - searchSize.Height) : startY;
                            System.Drawing.Point startPoint = new System.Drawing.Point(startX, startY);
                            searchRect = new Rectangle(startPoint, searchSize);
                            searchArea = Copy(image, searchRect);

                            var task1 = new Task(TrackColor);
                            task1.Start();

                            BitmapData data = image.LockBits(new Rectangle(0, 0, image.Width, image.Height),
                            ImageLockMode.ReadWrite, image.PixelFormat);

                            UnmanagedImage img = new UnmanagedImage(data);

                            RectanglesMarker marker = new RectanglesMarker(searchRect);
                            marker.MarkerColor = System.Drawing.Color.GreenYellow;
                            marker.ApplyInPlace(img);

                            image.UnlockBits(data);

                            ImageSource = Convert(image);
                            image.Dispose();
                        }
                    }
                }
            }

            using(DepthFrame depthFrame = comboFrame.DepthFrameReference.AcquireFrame())
            {
                if(depthFrame != null)
                {
                    FrameDescription depthFD = depthFrame.FrameDescription;
                    using (KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        if ((depthFD.Width * depthFD.Height) == (depthBuffer.Size / depthFD.BytesPerPixel))
                        {
                            lock (this)
                            {
                                depthArray.Initialize();
                                depthFrame.CopyFrameDataToArray(depthArray);
                                depthFrame.Dispose();

                                var task2 = new Task(ProcessDepth);
                                task2.Start();
                            }
                        }
                    }
                }
            }
//            CenterText = (center.X + ", " + center.Y + ", " + _depth);
            //CenterText = (float.IsNegativeInfinity(_x)) ? "Too close to sensor!" : (_x + ", " + _y + ", " + _depth);
            if (!float.IsNegativeInfinity(_x))
            {
                trackedPoint.Clear();
                trackedPoint.Add(new Point3D(_x, _y, _depth));
                HelixImage.Children.Remove(pointVisual);
                pointVisual.Points = trackedPoint;
                HelixImage.Children.Add(pointVisual);
            }
        }

        static public Bitmap Copy(Bitmap srcBitmap, Rectangle section)
        {
            // Create the new bitmap and associated graphics object
            Bitmap bmp = new Bitmap(section.Width, section.Height);
            Graphics g = Graphics.FromImage(bmp);

            // Draw the specified section of the source bitmap to the new one
            g.DrawImage(srcBitmap, 0, 0, section, GraphicsUnit.Pixel);

            // Clean up
            g.Dispose();

            // Return the bitmap
            return bmp;
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        public void RaisePropertyChanged(string propName)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propName));
            }
        }

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        //public ImageSource ImageSource
        //{
        //    get
        //    {
        //        return this.imageSource;
        //    }
        //}
        /// <summary>
        /// The <see cref="ImageSource" /> property's name.
        /// </summary>
        public const string ImageSourcePropertyName = "ImageSource";

        private BitmapSource imageSource = null;

        /// <summary>
        /// Sets and gets the ImageSource property.
        /// Changes to that property's value raise the PropertyChanged event. 
        /// </summary>
        public BitmapSource ImageSource
        {
            get
            {
                return imageSource;
            }

            set
            {
                if (imageSource == value)
                {
                    return;
                }

                imageSource = value;

                this.Dispatcher.Invoke((Action)(() =>
                {
                    ColorView.Source = imageSource;
                }));

                RaisePropertyChanged(ImageSourcePropertyName);
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
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

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.colorFrameReader != null)
            {
                // ColorFrameReder is IDisposable
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            mapCloud = true;
            if (this.colorBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                //encoder.Frames.Add(BitmapFrame.Create(this.colorBitmap));

                //string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                //string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                //string path = Path.Combine(myPhotos, "KinectScreenshot-Color-" + time + ".png");

                // write the new file to disk
                //try
                //{
                //    // FileStream is IDisposable
                //    using (FileStream fs = new FileStream(path, FileMode.Create))
                //    {
                //        encoder.Save(fs);
                //    }

                //    this.StatusText = string.Format(Properties.Resources.SavedScreenshotStatusTextFormat, path);
                //}
                //catch (IOException)
                //{
                //    this.StatusText = string.Format(Properties.Resources.FailedScreenshotStatusTextFormat, path);
                //}
            }
        }

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        lock (this)
                        {
                            Bitmap image = ImageToBitmap(colorFrame);
                            tracker.ComputeOrientation = false;

                            System.Drawing.Size searchSize = new System.Drawing.Size(175, 175);
                            int startX = center.X - searchSize.Width / 2;
                            int startY = center.Y - searchSize.Height / 2;
                            startX = (startX < 0) ? 0 : startX;
                            startY = (startY < 0) ? 0 : startY;
                            startX = ((startX + searchSize.Width) > image.Width) ? (image.Width - searchSize.Width) : startX;
                            startY = ((startY + searchSize.Height) > image.Height) ? (image.Height - searchSize.Height) : startY;
                            System.Drawing.Point startPoint = new System.Drawing.Point(startX, startY);
                            Rectangle searchRect = new Rectangle(startPoint, searchSize);
                            Bitmap searchArea = new Bitmap(searchSize.Width, searchSize.Height, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
                            searchArea = Copy(image, searchRect);

                            BitmapData searchData = searchArea.LockBits(new Rectangle(0, 0, searchArea.Width, searchArea.Height),
                                ImageLockMode.ReadWrite, searchArea.PixelFormat);
                            UnmanagedImage searchImg = new UnmanagedImage(searchData);
                            tracker.ProcessFrame(searchImg);
                            if (!tracker.TrackingObject.IsEmpty)
                            {
                                IntPoint dummyCenter = tracker.TrackingObject.Center;
                                center.X += (dummyCenter.X - searchSize.Width / 2);
                                center.Y += (dummyCenter.Y - searchSize.Height / 2);
                                CenterText = (center.X + ", " + center.Y);
                            }
                            searchArea.UnlockBits(searchData);

                            BitmapData data = image.LockBits(new Rectangle(0, 0, image.Width, image.Height),
                            ImageLockMode.ReadWrite, image.PixelFormat);
                            
                            UnmanagedImage img = new UnmanagedImage(data);

                            RectanglesMarker marker = new RectanglesMarker(searchRect);
                            marker.ApplyInPlace(img);

                            image.UnlockBits(data);
                            ImageSource = Convert(image);
                            image.Dispose();
                        }
                    }
                }
            }
        }

        public BitmapSource Convert(System.Drawing.Bitmap bitmap)
        {
            var bitmapData = bitmap.LockBits(
                new System.Drawing.Rectangle(0, 0, bitmap.Width, bitmap.Height),
                System.Drawing.Imaging.ImageLockMode.ReadOnly, bitmap.PixelFormat);

            var bitmapSource = BitmapSource.Create(
                bitmapData.Width, bitmapData.Height, 96, 96, PixelFormats.Bgr32, null,
                bitmapData.Scan0, bitmapData.Stride * bitmapData.Height, bitmapData.Stride);

            bitmap.UnlockBits(bitmapData);
            return bitmapSource;
        }

        public Bitmap ImageToBitmap(ColorFrame image)
        {
            FrameDescription fd = image.FrameDescription;
            int pixelLength = fd.Width * fd.Height * ((PixelFormats.Bgr32.BitsPerPixel + 7) / 8);

            byte[] pixeldata = new byte[pixelLength];
            if (image.RawColorImageFormat == ColorImageFormat.Bgra)
            {
                image.CopyRawFrameDataToArray(pixeldata);
            }
            else
            {
                image.CopyConvertedFrameDataToArray(pixeldata, ColorImageFormat.Bgra);
            }

            Bitmap bmap = new Bitmap(fd.Width, fd.Height, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

            BitmapData bmapdata = bmap.LockBits(new Rectangle(0, 0, bmap.Width, bmap.Height), ImageLockMode.WriteOnly, bmap.PixelFormat);

            Marshal.Copy(pixeldata, 0, bmapdata.Scan0, pixeldata.Length);
            
            bmap.UnlockBits(bmapdata);

            return bmap;
        }

        private Bitmap GetBitmap(BitmapSource source)
        {
            Bitmap bmp = new Bitmap(source.PixelWidth, source.PixelHeight, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
            BitmapData data = bmp.LockBits(
              new Rectangle(System.Drawing.Point.Empty, bmp.Size),
              ImageLockMode.WriteOnly,
              System.Drawing.Imaging.PixelFormat.Format32bppArgb);
            source.CopyPixels(
              Int32Rect.Empty,
              data.Scan0,
              data.Height * data.Stride,
              data.Stride);
            bmp.UnlockBits(data);
            return bmp;
        }

        public static Bitmap RescaleImage(System.Drawing.Image source, System.Drawing.Size size)
        {
            // 1st bullet, pixel format
            var bmp = new Bitmap(size.Width, size.Height, source.PixelFormat);
            // 2nd bullet, resolution
            bmp.SetResolution(source.HorizontalResolution, source.VerticalResolution);
            using (var gr = Graphics.FromImage(bmp))
            {
                // 3rd bullet, background
                gr.Clear(System.Drawing.Color.Transparent);
                // 4th bullet, interpolation
                gr.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.Low;
                gr.DrawImage(source, new Rectangle(0, 0, size.Width, size.Height));
            }
            return bmp;
        }

        [System.Runtime.InteropServices.DllImport("gdi32.dll")]
        public static extern bool DeleteObject(IntPtr hObject);

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        /// <summary>
        /// The <see cref="CenterText" /> property's name.
        /// </summary>
        public const string CenterTextPropertyName = "CenterText";

        private string centerText = "";

        /// <summary>
        /// Sets and gets the CenterText property.
        /// Changes to that property's value raise the PropertyChanged event. 
        /// </summary>
        public string CenterText
        {
            get
            {
                return centerText;
            }

            set
            {
                if (centerText == value)
                {
                    return;
                }

                centerText = value;
                RaisePropertyChanged(CenterTextPropertyName);
            }
        }
    }
}
