//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Windows.Forms;
//using System.Runtime.InteropServices; // Add this line
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using K4AdotNet.Sensor; // Orbbec Femto package
//using System;
//using System.IO;

//namespace ARSandbox
//{
//    public class ARSandboxComponent : GH_Component
//    {
//        /// <summary>
//        /// Each implementation of GH_Component must provide a public 
//        /// constructor without any arguments.
//        /// Category represents the Tab in which the component will appear, 
//        /// Subcategory the panel. If you use non-existing tab or panel names, 
//        /// new tabs/panels will automatically be created.
//        /// </summary>
//        /// 

//        // Sensor variables
//        private Device orbbecDevice = null;
//        private PointCloud pointCloud = null;
//        private Bitmap depthBitmap = null;  // Bitmap for depth image
//        private bool isSensorConnected = false;
//        public static double unitsMultiplier = 1;
//        private CameraCalibration cameraCalibration;
//        public ARSandboxComponent()
//          : base("ARSandboxComponent", "OrbbecSandbox",
//            "To view orbbec depth on rhinoviewport",
//            "ARSandbox", "Subcategory")
//        {
//        }

//        /// <summary>
//        /// Registers all the input parameters for this component.
//        /// </summary>
//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start Sensor", "S", "Start Orbbec sensor", GH_ParamAccess.item, false);
//            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
//            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
//            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
//            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image", GH_ParamAccess.item);  // Add depth image output
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            double refreshRate = 30.0;

//            // Get the inputs
//            DA.GetData(0, ref startSensor);
//            DA.GetData(1, ref refreshRate);

//            if (startSensor && !isSensorConnected)
//            {
//                StartOrbbecFemto();
//            }

//            if (isSensorConnected && orbbecDevice != null)
//            {
//                CapturePointCloud();
//                DA.SetDataList(0, ConvertToRhinoPoints(pointCloud));

//                // Output the depth image as a bitmap
//                DA.SetData(3, depthBitmap);
//            }

//            ScheduleSolve();
//        }

//        private void StartOrbbecFemto()
//        {
//            try
//            {
//                orbbecDevice = Device.Open(); // Opens the Orbbec Femto Bolt device

//                // Create a device configuration
//                var config = new DeviceConfiguration
//                {
//                    DepthMode = DepthMode.NarrowViewUnbinned, // Set to desired depth mode (choose appropriate mode)
//                    ColorResolution = ColorResolution.R720p, // Set color resolution if needed
//                    CameraFps = FrameRate.Thirty,
//                };

//                // Start the camera with the configuration
//                orbbecDevice.StartCameras(config);
//                isSensorConnected = true;
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
//                isSensorConnected = false;
//            }
//        }


//        private void CapturePointCloud()
//        {
//            Capture capture = null;
//            try
//            {
//                if (orbbecDevice != null && orbbecDevice.TryGetCapture(out capture, TimeSpan.FromMilliseconds(1000)))
//                {
//                    var depthImage = capture.DepthImage; // Get the depth image

//                    if (depthImage != null)
//                    {
//                        int width = depthImage.WidthPixels;
//                        int height = depthImage.HeightPixels;

//                        // Use ushort[] if depth data is in ushort format
//                        //int[] depthData = new int[width * height];
//                        short[] depthData = new short[width * height];
//                        IntPtr depthDataPtr = depthImage.Buffer;

//                        // Validate the pointer
//                        if (depthDataPtr != IntPtr.Zero && depthData.Length == (width * height))
//                        {
//                            //Marshal.Copy(depthDataPtr, depthData, 0, depthData.Length);
//                            Marshal.Copy((IntPtr)depthDataPtr, (short[])(object)depthData, 0, depthData.Length);
//                        }
//                        else
//                        {
//                            throw new InvalidOperationException("Invalid depth data pointer or size mismatch.");
//                        }

//                        // Copy the data to the ushort array
//                        //Marshal.Copy(depthDataPtr, depthData, 0, depthData.Length);

//                        pointCloud = new PointCloud();
//                        depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);

//                        //for (int y = 0; y < height; y++)
//                        //{
//                        //    for (int x = 0; x < width; x++)
//                        //    {
//                        //        ushort depthValue = (ushort)depthData[y * width + x]; // Get the ushort value directly

//                        //        // Check for valid depth values
//                        //        if (depthValue > 0) // Valid depth check
//                        //        {
//                        //            // Convert depth to 3D point
//                        //            var point = TransformDepthToPoint(x, y, depthValue);
//                        //            pointCloud.Add(point);
//                        //        }
//                        //    }
//                        //}
//                        // Access intrinsic parameters for depth camera (or color camera)

//                        double fx = cameraCalibration.Intrinsics.Parameters.Fx;
//                        double fy = cameraCalibration.Intrinsics.Parameters.Fy;
//                        double cx = cameraCalibration.Intrinsics.Parameters.Cx;
//                        double cy = cameraCalibration.Intrinsics.Parameters.Cy;


//                        // Define the file path (can be customized as needed)
//                        string filePath = @"camera_parameters.txt";  // This will save in the current working directory

//                        try
//                        {
//                            // Log the camera parameters
//                            using (StreamWriter writer = new StreamWriter(filePath, append: true))  // 'append: true' to avoid overwriting
//                            {
//                                writer.WriteLine("Camera Calibration Parameters:");
//                                writer.WriteLine($"Fx: {fx}");
//                                writer.WriteLine($"Fy: {fy}");
//                                writer.WriteLine($"Cx: {cx}");
//                                writer.WriteLine($"Cy: {cy}");
//                                writer.WriteLine($"Log Time: {DateTime.Now}");
//                                writer.WriteLine("-------------------------------");
//                            }

//                            Console.WriteLine("Camera parameters logged successfully.");
//                        }
//                        catch (Exception ex)
//                        {
//                            Console.WriteLine($"An error occurred while logging camera parameters: {ex.Message}");
//                        }


//                        for (int y = 0; y < height; y++)
//                        {
//                            for (int x = 0; x < width; x++)
//                            {
//                                ushort depthvalue = (ushort)depthData[y * width + x]; // get the ushort value directly

//                                // check for valid depth values
//                                if (depthvalue > 0) // valid depth check
//                                {
//                                    // convert depth to 3d point
//                                    //var point = transformdepthtopoint(x, y, depthvalue);
//                                    var point = TransformDepthToPoint(x, y, depthvalue, fx, fy, cx, cy);

//                                    pointCloud.Add(point);
//                                    // Set pixel value in the depth bitmap (scaling to 0-255 grayscale)
//                                    int intensity = (int)(depthvalue * 255.0 / 2000.0); // Adjust depth scale if needed
//                                    intensity = Math.Min(Math.Max(intensity, 0), 255);
//                                    depthBitmap.SetPixel(x, y, Color.FromArgb(intensity, intensity, intensity));
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//            finally
//            {
//                capture?.Dispose();
//            }
//        }






//        //private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//        //private Point3d TransformDepthToPoint(int x, int y, int depth)

//        private Point3d TransformDepthToPoint(int x, int y, ushort depth, double fxX, double fyy, double cxx, double cyy)

//        {
//            // Replace this with the actual transformation logic based on the Orbbec Femto calibration
//            double fx = 2225.71; // Focal length in x
//            double fy = 2231.60; // Focal length in y
//            double cx = 1935.19; // Principal point x
//            double cy = 1104.50; // Principal point y
//            // Replace this with the actual transformation logic based on the Orbbec Femto calibration
////double fx = 280; // focal length in x
////double fy = 280; // focal length in y
////double cx = 220; // principal point x
////double cy = 240; // principal point



//            double z = depth * 0.001; // Convert to meters
//            double pointX = (x - cx) * z / fx;
//            double pointY = (y - cy) * z / fy;

//            return new Point3d(pointX, pointY, z);
//        }

//        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
//        {
//            List<Point3d> rhinoPoints = new List<Point3d>();

//            // Assuming PointCloud class has a method to access point data
//            foreach (var point in pointCloud.GetPoints()) // Replace with the actual method to get points
//            {
//                rhinoPoints.Add(new Point3d(point.X, point.Y, point.Z));
//            }

//            return rhinoPoints;
//        }


//        protected override void ExpireDownStreamObjects()
//        {
//            base.ExpireDownStreamObjects();
//        }

//        public override void DrawViewportWires(IGH_PreviewArgs args)
//        {
//            if (pointCloud != null)
//            {
//                args.Display.DrawPointCloud(pointCloud, 3);
//            }

//            if (depthBitmap != null)
//            {
//                try
//                {
//                    Rhino.Display.DisplayBitmap displayBitmap = new Rhino.Display.DisplayBitmap(depthBitmap);
//                    int x = 0;
//                    int y = 0;
//                    args.Display.DrawBitmap(displayBitmap, x, y);
//                }
//                catch (ExternalException ex)
//                {
//                    // Handle exception, log error, or provide fallback
//                    Console.WriteLine("Error creating DisplayBitmap: " + ex.Message);
//                }
//            }
//        }

//        public override BoundingBox ClippingBox
//        {
//            get
//            {
//                if (pointCloud != null)
//                {
//                    BoundingBox bbox = new BoundingBox();

//                    // Replace 'GetPoints()' with the actual method that retrieves the points from the PointCloud
//                    foreach (var point in pointCloud.GetPoints()) // Adjust according to your PointCloud class
//                    {
//                        bbox.Union(new Point3d(point.X, point.Y, point.Z));
//                    }
//                    return bbox;
//                }
//                return BoundingBox.Empty;
//            }
//        }


//        public override void RemovedFromDocument(GH_Document document)
//        {
//            if (orbbecDevice != null)
//            {
//                orbbecDevice.StopCameras();
//                orbbecDevice.Dispose();
//            }
//            base.RemovedFromDocument(document);
//        }

//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution((int)(1000.0 / 30.0), doc =>
//            {
//                ExpireSolution(false);
//            });
//        }

//        protected override Bitmap Icon => null;

//        /// <summary>
//        /// Each component must have a unique Guid to identify it. 
//        /// It is vital this Guid doesn't change otherwise old ghx files 
//        /// that use the old ID will partially fail during loading.
//        /// </summary>
//        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
//    }
//}

//version2

//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Windows.Forms;
//using System.Runtime.InteropServices;
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using K4AdotNet.Sensor;
//using System.IO;

//namespace ARSandbox
//{
//    public class ARSandboxComponent : GH_Component
//    {
//        private Device orbbecDevice = null;
//        private PointCloud pointCloud = null;
//        private Bitmap depthBitmap = null;
//        private bool isSensorConnected = false;
//        private double fx, fy, cx, cy;
//        private CameraCalibration cameraCalibration;

//        public ARSandboxComponent()
//          : base("ARSandboxComponent", "OrbbecSandbox",
//            "To view Orbbec depth on Rhino",
//            "ARSandbox", "Subcategory")
//        {
//        }

//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start", "S", "Start Orbbec sensor", GH_ParamAccess.item, false);
//            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
//            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
//            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
//            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image", GH_ParamAccess.item);
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            double refreshRate = 30.0;

//            DA.GetData(0, ref startSensor);
//            DA.GetData(1, ref refreshRate);

//            if (startSensor && !isSensorConnected)
//            {
//                StartOrbbecFemto();
//            }

//            if (isSensorConnected && orbbecDevice != null)
//            {
//                CapturePointCloud();
//                DA.SetDataList(0, ConvertToRhinoPoints(pointCloud));
//                DA.SetData(3, depthBitmap);
//            }

//            ScheduleSolve();
//        }

//        private void StartOrbbecFemto()
//        {
//            try
//            {
//                orbbecDevice = Device.Open();
//                var config = new DeviceConfiguration
//                {
//                    DepthMode = DepthMode.NarrowViewUnbinned,
//                    ColorResolution = ColorResolution.R720p,
//                    CameraFps = FrameRate.Thirty,
//                };

//                orbbecDevice.StartCameras(config);
//                //cameraCalibration = orbbecDevice.GetCalibration();
//                //fx = cameraCalibration.Intrinsics.Parameters.Fx;
//                //fy = cameraCalibration.Intrinsics.Parameters.Fy;
//                //cx = cameraCalibration.Intrinsics.Parameters.Cx;
//                //cy = cameraCalibration.Intrinsics.Parameters.Cy;
//                double fx = 2225.71; // Focal length in x
//                double fy = 2231.60; // Focal length in y
//                double cx = 1935.19; // Principal point x
//                double cy = 1104.50; // Principal point y

//                isSensorConnected = true;
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
//                isSensorConnected = false;
//            }
//        }
//        private int Clamp(int value, int min, int max)
//        {
//            if (value < min) return min;
//            if (value > max) return max;
//            return value;
//        }


//        //private void CapturePointCloud()
//        //{
//        //    Capture capture = null;
//        //    try
//        //    {
//        //        if (orbbecDevice != null && orbbecDevice.TryGetCapture(out capture, TimeSpan.FromMilliseconds(1000)))
//        //        {
//        //            var depthImage = capture.DepthImage;

//        //            if (depthImage != null)
//        //            {
//        //                int width = depthImage.WidthPixels;
//        //                int height = depthImage.HeightPixels;
//        //                short[] depthData = new short[width * height];
//        //                Marshal.Copy(depthImage.Buffer, depthData, 0, depthData.Length);

//        //                pointCloud = new PointCloud();
//        //                byte[] pixelBuffer = new byte[width * height * 3];

//        //                for (int y = 0; y < height; y++)
//        //                {
//        //                    for (int x = 0; x < width; x++)
//        //                    {
//        //                        int index = y * width + x;
//        //                        ushort depthValue = (ushort)depthData[index];

//        //                        if (depthValue > 0)
//        //                        {
//        //                            var point = TransformDepthToPoint(x, y, depthValue);
//        //                            pointCloud.Add(point);

//        //                            int intensity = Clamp((int)(depthValue * 255.0 / 2000.0), 0, 255);

//        //                            pixelBuffer[index * 3 + 0] = (byte)intensity;
//        //                            pixelBuffer[index * 3 + 1] = (byte)intensity;
//        //                            pixelBuffer[index * 3 + 2] = (byte)intensity;
//        //                        }
//        //                    }
//        //                }

//        //                depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//        //                BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//        //                Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//        //                depthBitmap.UnlockBits(bitmapData);
//        //            }
//        //        }
//        //    }
//        //    finally
//        //    {
//        //        capture?.Dispose();
//        //    }
//        //}


//        private void CapturePointCloud()
//        {
//            Capture capture = null;
//            try
//            {
//                if (orbbecDevice != null && orbbecDevice.TryGetCapture(out capture, TimeSpan.FromMilliseconds(1000)))
//                {
//                    var depthImage = capture.DepthImage;
//                    if (depthImage != null)
//                    {
//                        int width = depthImage.WidthPixels;
//                        int height = depthImage.HeightPixels;
//                        short[] depthData = new short[width * height];
//                        Marshal.Copy(depthImage.Buffer, depthData, 0, depthData.Length);

//                        pointCloud = new PointCloud();
//                        byte[] pixelBuffer = new byte[width * height * 3];

//                        for (int y = 0; y < height; y++)
//                        {
//                            for (int x = 0; x < width; x++)
//                            {
//                                int index = y * width + x;
//                                ushort depthValue = (ushort)depthData[index];

//                                if (depthValue > 0)
//                                {
//                                    // Transform depth to 3D point
//                                    var point = TransformDepthToPoint(x, y, depthValue);
//                                    pointCloud.Add(point);

//                                    // Map depth to color and update buffer
//                                    Color color = MapDepthToColor(depthValue);
//                                    pixelBuffer[index * 3 + 0] = color.B; // Blue channel
//                                    pixelBuffer[index * 3 + 1] = color.G; // Green channel
//                                    pixelBuffer[index * 3 + 2] = color.R; // Red channel
//                                }
//                            }
//                        }

//                        // Create a Bitmap from the pixel buffer
//                        if (depthBitmap == null || depthBitmap.Width != width || depthBitmap.Height != height)
//                        {
//                            depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//                        }
//                        BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//                        Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//                        depthBitmap.UnlockBits(bitmapData);
//                    }
//                }
//            }
//            finally
//            {
//                capture?.Dispose();
//            }
//        }


//        //private Color MapDepthToColor(ushort depth)
//        //{
//        //    ushort minDepth = 300; // Minimum sensor range
//        //    ushort maxDepth = 2000; // Maximum sensor range

//        //    // Clamp and normalize depth within the range
//        //    depth = (ushort)Clamp(depth, minDepth, maxDepth);
//        //    double normalizedDepth = (double)(depth - minDepth) / (maxDepth - minDepth);

//        //    // Map normalized depth to a color gradient (blue to red)
//        //    int r = (int)(normalizedDepth * 255);
//        //    int g = (int)((1.0 - Math.Abs(normalizedDepth - 0.5) * 2) * 255);
//        //    int b = (int)((1.0 - normalizedDepth) * 255);

//        //    return Color.FromArgb(r, g, b);
//        //}
//        private Color MapDepthToColor(ushort depth)
//        {
//            // Convert the 16-bit depth value to an 8-bit range by right-shifting
//            // This will map higher depth values to lighter colors and lower to darker
//            byte intensity = (byte)(depth >> 8);  // Right-shift to fit into 8-bit range

//            // Apply the same intensity across RGB channels for grayscale output
//            return Color.FromArgb(intensity, intensity, intensity);
//        }


//        private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//        {
//            double z = depth * 0.001;
//            double pointX = (x - cx) * z / fx;
//            double pointY = (y - cy) * z / fy;
//            return new Point3d(pointX, pointY, z);
//        }

//        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
//        {
//            List<Point3d> rhinoPoints = new List<Point3d>();
//            foreach (var point in pointCloud.GetPoints())
//            {
//                rhinoPoints.Add(new Point3d(point.X, point.Y, point.Z));
//            }
//            return rhinoPoints;
//        }

//        protected override void ExpireDownStreamObjects()
//        {
//            base.ExpireDownStreamObjects();
//        }

//        public override void DrawViewportWires(IGH_PreviewArgs args)
//        {
//            if (pointCloud != null)
//            {
//                args.Display.DrawPointCloud(pointCloud, 3);
//            }

//            if (depthBitmap != null)
//            {
//                try
//                {
//                    Rhino.Display.DisplayBitmap displayBitmap = new Rhino.Display.DisplayBitmap(depthBitmap);
//                    args.Display.DrawBitmap(displayBitmap, 0, 0);
//                }
//                catch (ExternalException ex)
//                {
//                    Console.WriteLine("Error creating DisplayBitmap: " + ex.Message);
//                }
//            }
//        }

//        public override BoundingBox ClippingBox
//        {
//            get
//            {
//                if (pointCloud != null)
//                {
//                    BoundingBox bbox = new BoundingBox();
//                    foreach (var point in pointCloud.GetPoints())
//                    {
//                        bbox.Union(new Point3d(point.X, point.Y, point.Z));
//                    }
//                    return bbox;
//                }
//                return BoundingBox.Empty;
//            }
//        }

//        public override void RemovedFromDocument(GH_Document document)
//        {
//            orbbecDevice?.StopCameras();
//            orbbecDevice?.Dispose();
//            base.RemovedFromDocument(document);
//        }

//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution((int)(1000.0 / 30.0), doc =>
//            {
//                ExpireSolution(false);
//            });
//        }

//        protected override Bitmap Icon => null;

//        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
//    }
//}


/////////////////////////////////////Version 3
///
//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Runtime.InteropServices;
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using K4AdotNet.Sensor;

//namespace ARSandbox
//{
//    public class ARSandboxComponent : GH_Component
//    {
//        private Device orbbecDevice = null;
//        private PointCloud pointCloud = null;
//        private Bitmap depthBitmap = null;
//        private bool isSensorConnected = false;
//        private double fx = 2225.71, fy = 2231.60, cx = 1935.19, cy = 1104.50;

//        public ARSandboxComponent()
//          : base("ARSandboxComponent", "OrbbecSandbox",
//            "To view Orbbec depth on RhinoViewport",
//            "ARSandbox", "Subcategory")
//        {
//        }

//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start", "S", "Start Orbbec sensor", GH_ParamAccess.item, true);
//            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
//            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
//            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
//            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image with color mapping", GH_ParamAccess.item);
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            double refreshRate = 30.0;

//            DA.GetData(0, ref startSensor);
//            DA.GetData(1, ref refreshRate);

//            if (startSensor && !isSensorConnected)
//            {
//                StartOrbbecFemto();
//            }

//            if (isSensorConnected && orbbecDevice != null)
//            {
//                CapturePointCloud();
//                DA.SetDataList(0, ConvertToRhinoPoints(pointCloud));
//                DA.SetData(1, GenerateMeshFromPointCloud());
//                DA.SetDataList(2, GenerateContours());

//                // Check if depthBitmap is being generated
//                if (depthBitmap != null)
//                {
//                    DA.SetData(3, depthBitmap);
//                }
//                else
//                {
//                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Depth image is not generated.");
//                }
//            }

//            ScheduleSolve();
//        }

//        private void StartOrbbecFemto()
//        {
//            try
//            {
//                orbbecDevice = Device.Open();
//                var config = new DeviceConfiguration
//                {
//                    DepthMode = DepthMode.NarrowViewUnbinned,
//                    ColorResolution = ColorResolution.R720p,
//                    CameraFps = FrameRate.Thirty,
//                };

//                orbbecDevice.StartCameras(config);
//                isSensorConnected = true;
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
//                isSensorConnected = false;
//            }
//        }

//        private void CapturePointCloud()
//        {
//            Capture capture = null;
//            try
//            {
//                if (orbbecDevice != null && orbbecDevice.TryGetCapture(out capture, TimeSpan.FromMilliseconds(1000)))
//                {
//                    var depthImage = capture.DepthImage;
//                    if (depthImage != null)
//                    {
//                        int width = depthImage.WidthPixels;
//                        int height = depthImage.HeightPixels;
//                        short[] depthData = new short[width * height];
//                        Marshal.Copy(depthImage.Buffer, depthData, 0, depthData.Length);

//                        pointCloud = new PointCloud();
//                        byte[] pixelBuffer = new byte[width * height * 3];

//                        for (int y = 0; y < height; y++)
//                        {
//                            for (int x = 0; x < width; x++)
//                            {
//                                int index = y * width + x;
//                                ushort depthValue = (ushort)depthData[index];

//                                if (depthValue > 0)
//                                {
//                                    var point = TransformDepthToPoint(x, y, depthValue);
//                                    pointCloud.Add(point);

//                                    Color color = MapDepthToColor(depthValue);
//                                    pixelBuffer[index * 3 + 0] = color.B;
//                                    pixelBuffer[index * 3 + 1] = color.G;
//                                    pixelBuffer[index * 3 + 2] = color.R;
//                                }
//                            }
//                        }

//                        // Create the Bitmap and set pixels from pixelBuffer
//                        depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//                        BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//                        Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//                        depthBitmap.UnlockBits(bitmapData);
//                    }
//                }
//            }
//            finally
//            {
//                capture?.Dispose();
//            }
//        }
//        // Method to convert point cloud to Rhino points
//        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
//        {
//            var rhinoPoints = new List<Point3d>();
//            foreach (var point in pointCloud.GetPoints())
//            {
//                rhinoPoints.Add(new Point3d(point.X, point.Y, point.Z));
//            }
//            return rhinoPoints;
//        }

//        // Method to generate a mesh from the point cloud
//        private Mesh GenerateMeshFromPointCloud()
//        {
//            var mesh = new Mesh();
//            foreach (var point in pointCloud.GetPoints())
//            {
//                mesh.Vertices.Add(point.X, point.Y, point.Z);
//            }

//            // Simple mesh creation logic – additional processing may be needed here
//            mesh.Faces.AddFace(0, 1, 2, 3); // Example face, update based on your data
//            mesh.Normals.ComputeNormals();
//            mesh.Compact();
//            return mesh;
//        }

//        // Method to generate contours from the point cloud
//        private List<Curve> GenerateContours()
//        {
//            var contours = new List<Curve>();

//            // Example: generate horizontal contours for every 10 units in Z-direction
//            double contourInterval = 10.0;
//            for (double z = pointCloud.GetBoundingBox(true).Min.Z;
//                 z <= pointCloud.GetBoundingBox(true).Max.Z;
//                 z += contourInterval)
//            {
//                var contourPoints = new List<Point3d>();
//                foreach (var point in pointCloud.GetPoints())
//                {
//                    if (Math.Abs(point.Z - z) < contourInterval / 2)
//                    {
//                        contourPoints.Add(new Point3d(point.X, point.Y, z));
//                    }
//                }

//                if (contourPoints.Count > 1)
//                {
//                    var polyline = new Polyline(contourPoints);
//                    contours.Add(polyline.ToNurbsCurve());
//                }
//            }
//            return contours;
//        }
//        private Color MapDepthToColor(ushort depth)
//        {
//            int intensity = (depth / 256) % 256;
//            return Color.FromArgb(255, intensity, 255 - intensity, 128);
//        }

//        private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//        {
//            double z = depth * 0.001;
//            double pointX = (x - cx) * z / fx;
//            double pointY = (y - cy) * z / fy;
//            return new Point3d(pointX, pointY, z);
//        }

//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution((int)(1000.0 / 30.0), doc => { ExpireSolution(false); });
//        }

//        public override BoundingBox ClippingBox
//        {
//            get
//            {
//                if (pointCloud != null)
//                {
//                    BoundingBox bbox = new BoundingBox();
//                    foreach (var point in pointCloud.GetPoints())
//                    {
//                        bbox.Union(new Point3d(point.X, point.Y, point.Z));
//                    }
//                    return bbox;
//                }
//                return BoundingBox.Empty;
//            }
//        }

//        public override void RemovedFromDocument(GH_Document document)
//        {
//            orbbecDevice?.StopCameras();
//            orbbecDevice?.Dispose();
//            base.RemovedFromDocument(document);
//        }

//        protected override Bitmap Icon => null;
//        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
//    }
//}

////version 5 shows dep
//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Windows.Forms;
//using System.Runtime.InteropServices;
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using K4AdotNet.Sensor;
//using System.IO;
//using Rhino.Runtime;
//using System.Numerics;
//using static System.Windows.Forms.VisualStyles.VisualStyleElement.ToolTip;

//namespace ARSandbox
//{
//    public class ARSandboxComponent : GH_Component
//    {
//        private Device orbbecDevice = null;
//        private PointCloud pointCloud = null;
//        private Bitmap depthBitmap = null;
//        private bool isSensorConnected = false;
//        private double fx, fy, cx, cy;
//        private CameraCalibration cameraCalibration;


//        public ARSandboxComponent()
//          : base("ARSandboxComponent", "OrbbecSandbox",
//            "To view Orbbec depth on Rhino",
//            "ARSandbox", "Subcategory")
//        {
//        }

//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start", "S", "Start Orbbec sensor", GH_ParamAccess.item, false);
//            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
//            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
//            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
//            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image", GH_ParamAccess.item);
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            double refreshRate = 30.0;

//            DA.GetData(0, ref startSensor);
//            DA.GetData(1, ref refreshRate);

//            if (startSensor && !isSensorConnected)
//            {
//                StartOrbbecFemto();
//            }

//            if (isSensorConnected && orbbecDevice != null)
//            {
//                CapturePointCloud();
//                DA.SetDataList(0, ConvertToRhinoPoints(pointCloud));
//                DA.SetData(3, depthBitmap);
//            }

//            ScheduleSolve();
//        }

//        private void StartOrbbecFemto()
//        {
//            try
//            {
//                orbbecDevice = Device.Open();
//                var config = new DeviceConfiguration
//                {
//                    //DepthMode = DepthMode.NarrowViewUnbinned,
//                    DepthMode = DepthMode.NarrowView2x2Binned,
//                    ColorResolution = ColorResolution.R720p,
//                    //CameraFps = FrameRate.Thirty,
//                    CameraFps = FrameRate.Thirty,
//                };

//                orbbecDevice.StartCameras(config);
//                //cameraCalibration = orbbecDevice.GetCalibration();
//                //fx = cameraCalibration.Intrinsics.Parameters.Fx;
//                //fy = cameraCalibration.Intrinsics.Parameters.Fy;
//                //cx = cameraCalibration.Intrinsics.Parameters.Cx;
//                //cy = cameraCalibration.Intrinsics.Parameters.Cy;
//                double fx = 2225.71; // Focal length in x
//                double fy = 2231.60; // Focal length in y
//                double cx = 1935.19; // Principal point x
//                double cy = 1104.50; // Principal point y

//                isSensorConnected = true;
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
//                isSensorConnected = false;
//            }
//        }
//        private int Clamp(int value, int min, int max)
//        {
//            if (value < min) return min;
//            if (value > max) return max;
//            return value;
//        }


//        //private void CapturePointCloud()
//        //{
//        //    Capture capture = null;
//        //    try
//        //    {
//        //        if (orbbecDevice != null && orbbecDevice.TryGetCapture(out capture, TimeSpan.FromMilliseconds(1000)))
//        //        {
//        //            var depthImage = capture.DepthImage;

//        //            if (depthImage != null)
//        //            {
//        //                int width = depthImage.WidthPixels;
//        //                int height = depthImage.HeightPixels;
//        //                short[] depthData = new short[width * height];
//        //                Marshal.Copy(depthImage.Buffer, depthData, 0, depthData.Length);

//        //                pointCloud = new PointCloud();
//        //                byte[] pixelBuffer = new byte[width * height * 3];

//        //                for (int y = 0; y < height; y++)
//        //                {
//        //                    for (int x = 0; x < width; x++)
//        //                    {
//        //                        int index = y * width + x;
//        //                        ushort depthValue = (ushort)depthData[index];

//        //                        if (depthValue > 0)
//        //                        {
//        //                            var point = TransformDepthToPoint(x, y, depthValue);
//        //                            pointCloud.Add(point);

//        //                            int intensity = Clamp((int)(depthValue * 255.0 / 2000.0), 0, 255);

//        //                            pixelBuffer[index * 3 + 0] = (byte)intensity;
//        //                            pixelBuffer[index * 3 + 1] = (byte)intensity;
//        //                            pixelBuffer[index * 3 + 2] = (byte)intensity;
//        //                        }
//        //                    }
//        //                }

//        //                depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//        //                BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//        //                Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//        //                depthBitmap.UnlockBits(bitmapData);
//        //            }
//        //        }
//        //    }
//        //    finally
//        //    {
//        //        capture?.Dispose();
//        //    }
//        //}



//        private void CapturePointCloud()
//        {
//            Capture capture = null;
//            try
//            {
//                if (orbbecDevice != null && orbbecDevice.TryGetCapture(out capture, TimeSpan.FromMilliseconds(1000)))
//                {
//                    var depthImage = capture.DepthImage;
//                    if (depthImage != null)
//                    {
//                        int width = depthImage.WidthPixels;
//                        int height = depthImage.HeightPixels;
//                        short[] depthData = new short[width * height];
//                        Marshal.Copy(depthImage.Buffer, depthData, 0, depthData.Length);

//                        pointCloud = new PointCloud();
//                        byte[] pixelBuffer = new byte[width * height * 3];

//                        for (int y = 0; y < height; y++)
//                        {
//                            for (int x = 0; x < width; x++)
//                            {
//                                int index = y * width + x;
//                                ushort depthValue = (ushort)depthData[index];

//                                if (depthValue > 0)
//                                {
//                                    // Map depth to color and update buffer
//                                    Color color = MapDepthToColor(depthValue);
//                                    // Transform depth to 3D point
//                                    var point = TransformDepthToPoint(x, y, depthValue);

//                                    pointCloud.Add(point,color);



//                                    pixelBuffer[index * 3 + 0] = color.B; // Blue channel
//                                    pixelBuffer[index * 3 + 1] = color.G; // Green channel
//                                    pixelBuffer[index * 3 + 2] = color.R; // Red channel
//                                }
//                            }
//                        }

//                        // Create a Bitmap from the pixel buffer
//                        if (depthBitmap == null || depthBitmap.Width != width || depthBitmap.Height != height)
//                        {
//                            depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//                        }
//                        BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//                        Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//                        depthBitmap.UnlockBits(bitmapData);
//                    }
//                }
//            }
//            finally
//            {
//                capture?.Dispose();
//            }
//        }


//        private Color MapDepthToColor(ushort depth)
//        {
//            //ushort minDepth = 300; // Minimum sensor range
//            //ushort maxDepth = 2000; // Maximum sensor range
//            ushort minDepth = 250; // Minimum sensor range
//            ushort maxDepth = 4650; // Maximum sensor range

//            // Clamp and normalize depth within the range
//            depth = (ushort)Clamp(depth, minDepth, maxDepth);
//            double normalizedDepth = (double)(depth - minDepth) / (maxDepth - minDepth);

//            // Map normalized depth to a color gradient (blue to red)
//            int r = (int)(normalizedDepth * 255);
//            int g = (int)((1.0 - Math.Abs(normalizedDepth - 0.5) * 2) * 255);
//            int b = (int)((1.0 - normalizedDepth) * 255);

//            return Color.FromArgb(r, g, b);
//        }
//        //private Color MapDepthToColor(ushort depth)
//        //{
//        //    // Convert the 16-bit depth value to an 8-bit range by right-shifting
//        //    // This will map higher depth values to lighter colors and lower to darker
//        //    byte intensity = (byte)(depth >> 8);  // Right-shift to fit into 8-bit range

//        //    // Apply the same intensity across RGB channels for grayscale output
//        //    return Color.FromArgb(intensity, intensity, intensity);
//        //}


//private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//{
//    double fx = 280; // focal length in x
//    double fy = 280; // focal length in y
//    double cx = 220; // principal point x
//    double cy = 240; // principal point
//                     //double fx = 2225.71; // Focal length in x
//                     //double fy = 2231.60; // Focal length in y
//                     //double cx = 1935.19; // Principal point x
//                     //double cy = 1104.50; // Principal point y
//    double z = depth * 0.001;
//    double pointX = (x - cx) * z / fx;
//    double pointY = (y - cy) * z / fy;
//    return new Point3d(pointX, pointY, z);
//}

//        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
//        {
//            List<Point3d> rhinoPoints = new List<Point3d>();
//            foreach (var point in pointCloud.GetPoints())
//            {
//                rhinoPoints.Add(new Point3d(point.X, point.Y, point.Z));
//            }
//            return rhinoPoints;
//        }

//        protected override void ExpireDownStreamObjects()
//        {
//            base.ExpireDownStreamObjects();
//        }

//        public override void DrawViewportWires(IGH_PreviewArgs args)
//        {
//            if (pointCloud != null)
//            {
//                args.Display.DrawPointCloud(pointCloud, 3);
//            }

//            if (depthBitmap != null)
//            {
//                try
//                {
//                    Rhino.Display.DisplayBitmap displayBitmap = new Rhino.Display.DisplayBitmap(depthBitmap);
//                    args.Display.DrawBitmap(displayBitmap, 0, 0);
//                }
//                catch (ExternalException ex)
//                {
//                    Console.WriteLine("Error creating DisplayBitmap: " + ex.Message);
//                }
//            }
//        }

//        public override BoundingBox ClippingBox
//        {
//            get
//            {
//                if (pointCloud != null)
//                {
//                    BoundingBox bbox = new BoundingBox();
//                    foreach (var point in pointCloud.GetPoints())
//                    {
//                        bbox.Union(new Point3d(point.X, point.Y, point.Z));
//                    }
//                    return bbox;
//                }
//                return BoundingBox.Empty;
//            }
//        }

//        public override void RemovedFromDocument(GH_Document document)
//        {
//            orbbecDevice?.StopCameras();
//            orbbecDevice?.Dispose();
//            base.RemovedFromDocument(document);
//        }

//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution((int)(1000.0 / 30.0), doc =>
//            {
//                ExpireSolution(false);
//            });
//        }

//        protected override Bitmap Icon => null;

//        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
//    }
//}

////version 5 shows dep
//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Windows.Forms;
//using System.Runtime.InteropServices;
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using K4AdotNet.Sensor;
//using System.IO;
//using Rhino.Runtime;
//using System.Numerics;
//using static System.Windows.Forms.VisualStyles.VisualStyleElement.ToolTip;

//namespace ARSandbox
//{
//    public class Raindrop
//    {
//        public Point3d Position;
//        public double Speed;
//        public double ImpactRadius;

//        public Raindrop(Point3d initialPosition, double speed, double impactRadius)
//        {
//            Position = initialPosition;
//            Speed = speed;
//            ImpactRadius = impactRadius;
//        }

//        public void Update()
//        {
//            // Simulate raindrop falling
//            Position.Z -= Speed;
//        }
//    }

//    public class ARSandboxComponent : GH_Component
//    {

//        private Device orbbecDevice = null;
//        private PointCloud pointCloud = null;
//        private Bitmap depthBitmap = null;
//        private bool isSensorConnected = false;
//        private double fx, fy, cx, cy;
//        private CameraCalibration cameraCalibration;
//        private double minHeight = 0.85;  // Default minimum height (in mm) 889mm
//        private double maxHeight = 1; // Default maximum height (in mm) 1066mm
//        private double[,] waterHeightMap;
//        private List<Raindrop> raindrops = new List<Raindrop>();


//        public ARSandboxComponent()
//          : base("ARSandboxComponent", "OrbbecSandbox",
//            "To view Orbbec depth on Rhino",
//            "ARSandbox", "Subcategory")
//        {
//        }

//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start", "S", "Start Orbbec sensor", GH_ParamAccess.item, false);
//            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
//            pManager.AddNumberParameter("MinHeight", "MinH", "Minimum depth height (in mm)", GH_ParamAccess.item, 0.8);
//            pManager.AddNumberParameter("MaxHeight", "MaxH", "Maximum depth height (in mm)", GH_ParamAccess.item, 1);
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
//            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
//            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
//            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image", GH_ParamAccess.item);
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            double refreshRate = 30.0;
//            double newMinHeight = minHeight;
//            double newMaxHeight = maxHeight;

//            DA.GetData(0, ref startSensor);
//            DA.GetData(1, ref refreshRate);
//            DA.GetData(2, ref newMinHeight);
//            DA.GetData(3, ref newMaxHeight);

//            // Only update if there is a change in the input parameters
//            if (newMinHeight != minHeight || newMaxHeight != maxHeight)
//            {
//                minHeight = newMinHeight;
//                maxHeight = newMaxHeight;
//            }

//            if (startSensor && !isSensorConnected)
//            {
//                StartOrbbecFemto();
//            }

//            if (isSensorConnected && orbbecDevice != null)
//            {

//                CapturePointCloud();
//                ApplyRaindropImpacts();
//                UpdateWaterFlow();
//                DA.SetDataList(0, ConvertToRhinoPoints(pointCloud));
//                DA.SetData(3, depthBitmap);
//            }

//            ScheduleSolve();
//        }
//        private void GenerateRain()
//        {
//            Random rnd = new Random();
//            for (int i = 0; i < 100; i++)
//            {
//                var x = rnd.Next(0, depthBitmap.Width);
//                var y = rnd.Next(0, depthBitmap.Height);
//                raindrops.Add(new Raindrop(new Point3d(x, y, maxHeight), 0.05, 0.1));
//            }
//        }
//        private void InitializeWaterHeightMap(int width, int height)
//        {
//            if (waterHeightMap == null || waterHeightMap.GetLength(0) != width || waterHeightMap.GetLength(1) != height)
//            {
//                waterHeightMap = new double[width, height];
//            }
//        }


//        private void UpdateWaterFlow()
//        {
//            // Iterate over water height map and spread water to neighboring cells
//            for (int x = 1; x < waterHeightMap.GetLength(0) - 1; x++)
//            {
//                for (int y = 1; y < waterHeightMap.GetLength(1) - 1; y++)
//                {
//                    double currentLevel = waterHeightMap[x, y];

//                    // Distribute water to neighboring cells based on height difference
//                    if (currentLevel > 0.0)
//                    {
//                        double flowRate = 0.1;
//                        double delta = flowRate * (currentLevel - waterHeightMap[x + 1, y]);
//                        waterHeightMap[x + 1, y] += delta;
//                        waterHeightMap[x, y] -= delta;

//                        delta = flowRate * (currentLevel - waterHeightMap[x - 1, y]);
//                        waterHeightMap[x - 1, y] += delta;
//                        waterHeightMap[x, y] -= delta;

//                        // Repeat for other directions as needed
//                    }
//                }
//            }
//        }

//        private void StartOrbbecFemto()
//        {
//            try
//            {
//                orbbecDevice = Device.Open();
//                if (orbbecDevice == null)
//                {
//                    Rhino.RhinoApp.WriteLine("Device not initialized.");
//                    return;
//                }

//                var config = new DeviceConfiguration
//                {
//                    DepthMode = DepthMode.NarrowViewUnbinned,
//                    ColorResolution = ColorResolution.R1080p,
//                    CameraFps = FrameRate.Thirty,
//                };

//                orbbecDevice.StartCameras(config);
//                double fx = 2225.71; // Focal length in x
//                double fy = 2231.60; // Focal length in y
//                double cx = 1935.19; // Principal point x
//                double cy = 1104.50; // Principal point y

//                isSensorConnected = true;
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
//                isSensorConnected = false;
//            }
//        }

//        private void CapturePointCloud()
//        {
//            Capture capture = null;
//            try
//            {
//                if (orbbecDevice != null && orbbecDevice.TryGetCapture(out capture, TimeSpan.FromMilliseconds(1000)))
//                {
//                    var depthImage = capture.DepthImage;
//                    if (depthImage != null)
//                    {
//                        int width = depthImage.WidthPixels;
//                        int height = depthImage.HeightPixels;
//                        short[] depthData = new short[width * height];
//                        InitializeWaterHeightMap(width, height);
//                        Marshal.Copy(depthImage.Buffer, depthData, 0, depthData.Length);

//                        pointCloud = new PointCloud();
//                        byte[] pixelBuffer = new byte[width * height * 3];

//                        for (int y = 0; y < height; y++)
//                        {
//                            for (int x = 0; x < width; x++)
//                            {
//                                int index = y * width + x;
//                                ushort depthValue = (ushort)depthData[index];

//                                if (depthValue > 0)
//                                {
//                                    // Convert depth value to real-world depth in mm (or cm)
//                                    double depthInMeters = depthValue * 0.001; // Convert to meters if depth is in mm

//                                    // Filter depth within the specified height range
//                                    //if (depthInMeters >= minHeight && depthInMeters <= maxHeight)
//                                    //{
//                                    //    // Map depth to color based on the range
//                                    //    Color color = MapDepthToColor(depthInMeters);

//                                    //    // Transform depth to 3D point
//                                    //    var point = TransformDepthToPoint(x, y, depthValue);

//                                    //    // Add point with color to point cloud
//                                    //    pointCloud.Add(point, color);

//                                    //    // Store color data for bitmap visualization
//                                    //    pixelBuffer[index * 3 + 0] = color.B; // Blue channel
//                                    //    pixelBuffer[index * 3 + 1] = color.G; // Green channel
//                                    //    pixelBuffer[index * 3 + 2] = color.R; // Red channel

//                                    //}
//                                    // Filter depth within the specified height range (only process water below the depth limit)
//                                    if (depthInMeters >= minHeight && depthInMeters <= maxHeight)
//                                    {
//                                        // Limit water depth: Ensure water effect stops at a particular depth (e.g., 0.9m)
//                                        double waterLevelDepthLimit = 0.9; // Set water depth limit (example value)
//                                        if (depthInMeters <= waterLevelDepthLimit)
//                                        {
//                                            // Map depth to color based on the range
//                                            Color color = MapDepthToColor(depthInMeters);

//                                            // Transform depth to 3D point
//                                            var point = TransformDepthToPoint(x, y, depthValue);

//                                            // Add point with color to point cloud
//                                            pointCloud.Add(point, color);

//                                            // Store color data for bitmap visualization
//                                            pixelBuffer[index * 3 + 0] = color.B; // Blue channel
//                                            pixelBuffer[index * 3 + 1] = color.G; // Green channel
//                                            pixelBuffer[index * 3 + 2] = color.R; // Red channel

//                                        }
//                                        else
//                                        {
//                                            waterHeightMap[x, y] = depthInMeters;
//                                        }
//                                    }
//                                }
//                            }
//                        }

//                        // Create a Bitmap from the pixel buffer
//                        if (depthBitmap == null || depthBitmap.Width != width || depthBitmap.Height != height)
//                        {
//                            depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//                        }
//                        BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//                        Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//                        depthBitmap.UnlockBits(bitmapData);
//                    }
//                }
//            }
//            finally
//            {
//                capture?.Dispose();
//            }
//        }
//        private void ApplyRaindropImpacts()
//        {
//            var dropsToRemove = new List<Raindrop>();

//            foreach (var raindrop in raindrops)
//            {
//                if (raindrop.Position.Z <= minHeight)
//                {
//                    int x = (int)raindrop.Position.X;
//                    int y = (int)raindrop.Position.Y;
//                    if (x >= 0 && y >= 0 && x < waterHeightMap.GetLength(0) && y < waterHeightMap.GetLength(1))
//                    {
//                        waterHeightMap[x, y] += raindrop.ImpactRadius;
//                    }

//                    dropsToRemove.Add(raindrop); // Mark for removal after the loop
//                }
//                else
//                {
//                    raindrop.Update();
//                }
//            }

//            foreach (var drop in dropsToRemove)
//            {
//                raindrops.Remove(drop);
//            }
//        }



//        // Method to map depth to color based on height range
//        private Color MapDepthToColor(double depthInMeters)
//        {
//            // Normalize the depth value to a color gradient based on the height range
//            double normalizedDepth = (depthInMeters - minHeight) / (maxHeight - minHeight);

//            // Clamp the normalized depth between 0 and 1
//            normalizedDepth = Clamp(normalizedDepth, 0.0, 1.0);

//            // Create a color gradient (you can adjust this for any color scheme you prefer)
//            int r = (int)(normalizedDepth * 255);
//            int g = (int)((1.0 - Math.Abs(normalizedDepth - 0.5) * 2) * 255);
//            int b = (int)((1.0 - normalizedDepth) * 255);

//            return Color.FromArgb(r, g, b);
//        }

//        // Method to clamp a value between a minimum and maximum
//        private double Clamp(double value, double min, double max)
//        {
//            return Math.Max(min, Math.Min(max, value));
//        }

//        // Transform depth value to 3D point in space (example)
//        private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//        {
//            //double fx = 280; // focal length in x
//            //double fy = 280; // focal length in y
//            //double cx = 220; // principal point x
//            //double cy = 240; // principal point
//            //double fx = 2225.71; // Focal length in x
//            //double fy = 2231.60; // Focal length in y
//            //double cx = 1935.19; // Principal point x
//            //double cy = 1104.50; // Principal point y
//            double fx = 322; // Focal length in x
//            double fy = 323; // Focal length in y
//            double cx = 280; // Principal point x
//            double cy = 160; // Principal point y

//            double z = depth * 0.001;
//            double pointX = (x - cx) * z / fx;
//            double pointY = (y - cy) * z / fy;
//            return new Point3d(pointX, pointY, z);
//        }


//        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
//        {
//            List<Point3d> rhinoPoints = new List<Point3d>();
//            foreach (var point in pointCloud.GetPoints())
//            {
//                rhinoPoints.Add(new Point3d(point.X, point.Y, point.Z));
//            }
//            return rhinoPoints;
//        }

//        protected override void ExpireDownStreamObjects()
//        {
//            base.ExpireDownStreamObjects();
//        }

//        public override void DrawViewportWires(IGH_PreviewArgs args)
//        {
//            if (pointCloud != null)
//            {
//                args.Display.DrawPointCloud(pointCloud, 3);
//            }

//            if (depthBitmap != null)
//            {
//                try
//                {
//                    Rhino.Display.DisplayBitmap displayBitmap = new Rhino.Display.DisplayBitmap(depthBitmap);
//                    args.Display.DrawBitmap(displayBitmap, 0, 0);
//                }
//                catch (ExternalException ex)
//                {
//                    Console.WriteLine("Error creating DisplayBitmap: " + ex.Message);
//                }
//            }
//            //if (depthBitmap != null)
//            //{
//            //    try
//            //    {
//            //        // Get the current viewport size
//            //        var viewportSize = args.Viewport.Size;

//            //        // Scale the bitmap to fit the viewport
//            //        Bitmap scaledBitmap = new Bitmap(viewportSize.Width, viewportSize.Height);
//            //        using (Graphics graphics = Graphics.FromImage(scaledBitmap))
//            //        {
//            //            graphics.Clear(Color.Black); // Set background to black
//            //            graphics.DrawImage(depthBitmap, new Rectangle(0, 0, viewportSize.Width, viewportSize.Height));
//            //        }

//            //        // Display the scaled bitmap
//            //        Rhino.Display.DisplayBitmap displayBitmap = new Rhino.Display.DisplayBitmap(scaledBitmap);
//            //        args.Display.DrawBitmap(displayBitmap, 0, 0);
//            //    }
//            //    catch (ExternalException ex)
//            //    {
//            //        Console.WriteLine("Error creating DisplayBitmap: " + ex.Message);
//            //    }
//            //}
//            if (depthBitmap != null && waterHeightMap != null)
//            {
//                Bitmap waterBitmap = new Bitmap(depthBitmap.Width, depthBitmap.Height);

//                for (int x = 0; x < waterHeightMap.GetLength(0); x++)
//                {
//                    for (int y = 0; y < waterHeightMap.GetLength(1); y++)
//                    {
//                        double waterLevel = waterHeightMap[x, y];
//                        if (waterLevel > 0)
//                        {
//                            int intensity = Math.Min(255, (int)(waterLevel * 500));
//                            Color waterColor = Color.FromArgb(intensity, 0, 0, 255);
//                            waterBitmap.SetPixel(x, y, waterColor);
//                        }
//                    }
//                }
//                Rhino.RhinoApp.WriteLine("Water level at center: " + waterHeightMap[depthBitmap.Width / 2, depthBitmap.Height / 2]);

//                Rhino.Display.DisplayBitmap displayWaterBitmap = new Rhino.Display.DisplayBitmap(waterBitmap);
//                args.Display.DrawBitmap(displayWaterBitmap, 0, 0);
//            }

//        }

//        public override BoundingBox ClippingBox
//        {
//            get
//            {
//                if (pointCloud != null)
//                {
//                    BoundingBox bbox = new BoundingBox();
//                    foreach (var point in pointCloud.GetPoints())
//                    {
//                        bbox.Union(new Point3d(point.X, point.Y, point.Z));
//                    }
//                    return bbox;
//                }
//                return BoundingBox.Empty;
//            }
//        }

//        public override void RemovedFromDocument(GH_Document document)
//        {
//            orbbecDevice?.StopCameras();
//            orbbecDevice?.Dispose();
//            base.RemovedFromDocument(document);
//        }

//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution((int)(1000.0 / 30.0), doc =>
//            {
//                ExpireSolution(false);
//            });
//        }

//        protected override Bitmap Icon => null;

//        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
//    }
//}


//version 5 shows dep
//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Windows.Forms;
//using System.Runtime.InteropServices;
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using K4AdotNet.Sensor;
//using System.IO;
//using Rhino.Runtime;
//using System.Numerics;
//using static System.Windows.Forms.VisualStyles.VisualStyleElement.ToolTip;

//namespace ARSandbox
//{
//    public class Raindrop
//    {
//        public Point3d Position;
//        public double Speed;
//        public double ImpactRadius;

//        public Raindrop(Point3d initialPosition, double speed, double impactRadius)
//        {
//            Position = initialPosition;
//            Speed = speed;
//            ImpactRadius = impactRadius;
//        }

//        public void Update()
//        {
//            // Simulate raindrop falling
//            Position.Z -= Speed;
//        }
//    }

//    public class ARSandboxComponent : GH_Component
//    {

//        private Device orbbecDevice = null;
//        private PointCloud pointCloud = null;
//        private Bitmap depthBitmap = null;
//        private bool isSensorConnected = false;
//        private double fx, fy, cx, cy;
//        private CameraCalibration cameraCalibration;
//        private double minHeight = 0.85;  // Default minimum height (in mm) 889mm
//        private double maxHeight = 1; // Default maximum height (in mm) 1066mm
//        private double[,] waterHeightMap;
//        private List<Raindrop> raindrops = new List<Raindrop>();


//        public ARSandboxComponent()
//          : base("ARSandboxComponent", "OrbbecSandbox",
//            "To view Orbbec depth on Rhino",
//            "ARSandbox", "Subcategory")
//        {
//        }

//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start", "S", "Start Orbbec sensor", GH_ParamAccess.item, false);
//            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
//            pManager.AddNumberParameter("MinHeight", "MinH", "Minimum depth height (in mm)", GH_ParamAccess.item, 0.8);
//            pManager.AddNumberParameter("MaxHeight", "MaxH", "Maximum depth height (in mm)", GH_ParamAccess.item, 1);
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
//            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
//            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
//            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image", GH_ParamAccess.item);
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            double refreshRate = 30.0;
//            double newMinHeight = minHeight;
//            double newMaxHeight = maxHeight;

//            DA.GetData(0, ref startSensor);
//            DA.GetData(1, ref refreshRate);
//            DA.GetData(2, ref newMinHeight);
//            DA.GetData(3, ref newMaxHeight);

//            // Only update if there is a change in the input parameters
//            if (newMinHeight != minHeight || newMaxHeight != maxHeight)
//            {
//                minHeight = newMinHeight;
//                maxHeight = newMaxHeight;
//            }

//            if (startSensor && !isSensorConnected)
//            {
//                StartOrbbecFemto();
//            }

//            if (isSensorConnected && orbbecDevice != null)
//            {

//                CapturePointCloud();
//                ApplyRaindropImpacts();
//                UpdateWaterFlow();
//                DA.SetDataList(0, ConvertToRhinoPoints(pointCloud));
//                DA.SetData(3, depthBitmap);
//            }

//            ScheduleSolve();
//        }
//        private void GenerateRain()
//        {
//            Random rnd = new Random();
//            for (int i = 0; i < 100; i++)
//            {
//                var x = rnd.Next(0, depthBitmap.Width);
//                var y = rnd.Next(0, depthBitmap.Height);
//                raindrops.Add(new Raindrop(new Point3d(x, y, maxHeight), 0.05, 0.1));
//            }
//        }
//        private void InitializeWaterHeightMap(int width, int height)
//        {
//            if (waterHeightMap == null || waterHeightMap.GetLength(0) != width || waterHeightMap.GetLength(1) != height)
//            {
//                waterHeightMap = new double[width, height];
//            }
//        }


//        private void UpdateWaterFlow()
//        {
//            // Iterate over water height map and spread water to neighboring cells
//            for (int x = 1; x < waterHeightMap.GetLength(0) - 1; x++)
//            {
//                for (int y = 1; y < waterHeightMap.GetLength(1) - 1; y++)
//                {
//                    double currentLevel = waterHeightMap[x, y];

//                    // Distribute water to neighboring cells based on height difference
//                    if (currentLevel > 0.0)
//                    {
//                        double flowRate = 0.1;
//                        double delta = flowRate * (currentLevel - waterHeightMap[x + 1, y]);
//                        waterHeightMap[x + 1, y] += delta;
//                        waterHeightMap[x, y] -= delta;

//                        delta = flowRate * (currentLevel - waterHeightMap[x - 1, y]);
//                        waterHeightMap[x - 1, y] += delta;
//                        waterHeightMap[x, y] -= delta;

//                        // Repeat for other directions as needed
//                    }
//                }
//            }
//        }

//        private void StartOrbbecFemto()
//        {
//            try
//            {
//                orbbecDevice = Device.Open();
//                if (orbbecDevice == null)
//                {
//                    Rhino.RhinoApp.WriteLine("Device not initialized.");
//                    return;
//                }

//                var config = new DeviceConfiguration
//                {
//                    DepthMode = DepthMode.NarrowViewUnbinned,
//                    ColorResolution = ColorResolution.R1080p,
//                    CameraFps = FrameRate.Thirty,
//                };

//                orbbecDevice.StartCameras(config);
//                double fx = 2225.71; // Focal length in x
//                double fy = 2231.60; // Focal length in y
//                double cx = 1935.19; // Principal point x
//                double cy = 1104.50; // Principal point y

//                isSensorConnected = true;
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
//                isSensorConnected = false;
//            }
//        }

//        private void CapturePointCloud()
//        {
//            Capture capture = null;
//            try
//            {
//                if (orbbecDevice != null && orbbecDevice.TryGetCapture(out capture, TimeSpan.FromMilliseconds(1000)))
//                {
//                    var depthImage = capture.DepthImage;
//                    if (depthImage != null)
//                    {
//                        int width = depthImage.WidthPixels;
//                        int height = depthImage.HeightPixels;
//                        short[] depthData = new short[width * height];
//                        //InitializeWaterHeightMap(width, height);
//                        Marshal.Copy(depthImage.Buffer, depthData, 0, depthData.Length);

//                        pointCloud = new PointCloud();
//                        byte[] pixelBuffer = new byte[width * height * 3];

//                        for (int y = 0; y < height; y++)
//                        {
//                            for (int x = 0; x < width; x++)
//                            {
//                                int index = y * width + x;
//                                ushort depthValue = (ushort)depthData[index];

//                                if (depthValue > 0)
//                                {
//                                    // Convert depth value to real-world depth in mm (or cm)
//                                    double depthInMeters = depthValue * 0.001; // Convert to meters if depth is in mm

//                                    // Filter depth within the specified height range
//                                    if (depthInMeters >= minHeight && depthInMeters <= maxHeight)
//                                    {
//                                        // Map depth to color based on the range
//                                        Color color = MapDepthToColor(depthInMeters);

//                                        // Transform depth to 3D point
//                                        var point = TransformDepthToPoint(x, y, depthValue);

//                                        // Add point with color to point cloud
//                                        pointCloud.Add(point, color);

//                                        // Store color data for bitmap visualization
//                                        pixelBuffer[index * 3 + 0] = color.B; // Blue channel
//                                        pixelBuffer[index * 3 + 1] = color.G; // Green channel
//                                        pixelBuffer[index * 3 + 2] = color.R; // Red channel

//                                    }
//                                    // Filter depth within the specified height range (only process water below the depth limit)
//                                    if (depthInMeters >= minHeight && depthInMeters <= maxHeight)
//                                    {
//                                        // Limit water depth: Ensure water effect stops at a particular depth (e.g., 0.9m)
//                                        double waterLevelDepthLimit = 0.9; // Set water depth limit (example value)
//                                        if (depthInMeters > waterLevelDepthLimit)
//                                        {

//                                            waterHeightMap[x, y] = depthInMeters;
//                                            // Add point with color to point cloud
//                                            //pointCloud.Add(point, color);

//                                            // Store color data for bitmap visualization
//                                            //pixelBuffer[index * 3 + 0] = color.B; // Blue channel
//                                            //pixelBuffer[index * 3 + 1] = color.G; // Green channel
//                                            //pixelBuffer[index * 3 + 2] = color.R; // Red channel

//                                        }

//                                    }
//                                }
//                            }
//                        }

//                        // Create a Bitmap from the pixel buffer
//                        if (depthBitmap == null || depthBitmap.Width != width || depthBitmap.Height != height)
//                        {
//                            depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//                        }
//                        BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//                        Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//                        depthBitmap.UnlockBits(bitmapData);
//                    }
//                }
//            }
//            finally
//            {
//                capture?.Dispose();
//            }
//        }
//        private void ApplyRaindropImpacts()
//        {
//            var dropsToRemove = new List<Raindrop>();

//            foreach (var raindrop in raindrops)
//            {
//                if (raindrop.Position.Z <= minHeight)
//                {
//                    int x = (int)raindrop.Position.X;
//                    int y = (int)raindrop.Position.Y;
//                    if (x >= 0 && y >= 0 && x < waterHeightMap.GetLength(0) && y < waterHeightMap.GetLength(1))
//                    {
//                        waterHeightMap[x, y] += raindrop.ImpactRadius;
//                    }

//                    dropsToRemove.Add(raindrop); // Mark for removal after the loop
//                }
//                else
//                {
//                    raindrop.Update();
//                }
//            }

//            foreach (var drop in dropsToRemove)
//            {
//                raindrops.Remove(drop);
//            }
//        }



//        // Method to map depth to color based on height range
//        private Color MapDepthToColor(double depthInMeters)
//        {
//            // Normalize the depth value to a color gradient based on the height range
//            double normalizedDepth = (depthInMeters - minHeight) / (maxHeight - minHeight);

//            // Clamp the normalized depth between 0 and 1
//            normalizedDepth = Clamp(normalizedDepth, 0.0, 1.0);

//            // Create a color gradient (you can adjust this for any color scheme you prefer)
//            int r = (int)(normalizedDepth * 255);
//            int g = (int)((1.0 - Math.Abs(normalizedDepth - 0.5) * 2) * 255);
//            int b = (int)((1.0 - normalizedDepth) * 255);

//            return Color.FromArgb(r, g, b);
//        }

//        // Method to clamp a value between a minimum and maximum
//        private double Clamp(double value, double min, double max)
//        {
//            return Math.Max(min, Math.Min(max, value));
//        }

//        // Transform depth value to 3D point in space (example)
//        private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//        {
//            //double fx = 280; // focal length in x
//            //double fy = 280; // focal length in y
//            //double cx = 220; // principal point x
//            //double cy = 240; // principal point
//            //double fx = 2225.71; // Focal length in x
//            //double fy = 2231.60; // Focal length in y
//            //double cx = 1935.19; // Principal point x
//            //double cy = 1104.50; // Principal point y
//            double fx = 322; // Focal length in x
//            double fy = 323; // Focal length in y
//            double cx = 280; // Principal point x
//            double cy = 160; // Principal point y

//            double z = depth * 0.001;
//            double pointX = (x - cx) * z / fx;
//            double pointY = (y - cy) * z / fy;
//            return new Point3d(pointX, pointY, z);
//        }


//        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
//        {
//            List<Point3d> rhinoPoints = new List<Point3d>();
//            foreach (var point in pointCloud.GetPoints())
//            {
//                rhinoPoints.Add(new Point3d(point.X, point.Y, point.Z));
//            }
//            return rhinoPoints;
//        }

//        protected override void ExpireDownStreamObjects()
//        {
//            base.ExpireDownStreamObjects();
//        }

//        public override void DrawViewportWires(IGH_PreviewArgs args)
//        {
//            if (pointCloud != null)
//            {
//                args.Display.DrawPointCloud(pointCloud, 3);
//            }

//            //if (depthBitmap != null)
//            //{
//            //    try
//            //    {
//            //        Rhino.Display.DisplayBitmap displayBitmap = new Rhino.Display.DisplayBitmap(depthBitmap);
//            //        args.Display.DrawBitmap(displayBitmap, 0, 0);
//            //    }
//            //    catch (ExternalException ex)
//            //    {
//            //        Console.WriteLine("Error creating DisplayBitmap: " + ex.Message);
//            //    }
//            //}
//            if (depthBitmap != null)
//            {
//                try
//                {
//                    // Get the current viewport size
//                    var viewportSize = args.Viewport.Size;

//                    // Scale the bitmap to fit the viewport
//                    Bitmap scaledBitmap = new Bitmap(viewportSize.Width, viewportSize.Height);
//                    using (Graphics graphics = Graphics.FromImage(scaledBitmap))
//                    {
//                        graphics.Clear(Color.Black); // Set background to black
//                        graphics.DrawImage(depthBitmap, new Rectangle(0, 0, viewportSize.Width, viewportSize.Height));
//                    }

//                    // Display the scaled bitmap
//                    Rhino.Display.DisplayBitmap displayBitmap = new Rhino.Display.DisplayBitmap(scaledBitmap);
//                    args.Display.DrawBitmap(displayBitmap, 0, 0);
//                }
//                catch (ExternalException ex)
//                {
//                    Console.WriteLine("Error creating DisplayBitmap: " + ex.Message);
//                }
//            }
//            //if (depthBitmap != null && waterHeightMap != null)
//            //{
//            //    Bitmap waterBitmap = new Bitmap(depthBitmap.Width, depthBitmap.Height);

//            //    for (int x = 0; x < waterHeightMap.GetLength(0); x++)
//            //    {
//            //        for (int y = 0; y < waterHeightMap.GetLength(1); y++)
//            //        {
//            //            double waterLevel = waterHeightMap[x, y];
//            //            if (waterLevel > 0)
//            //            {
//            //                int intensity = Math.Min(255, (int)(waterLevel * 100));
//            //                Color waterColor = Color.FromArgb(intensity, 0, 0, 255);
//            //                waterBitmap.SetPixel(x, y, waterColor);
//            //            }
//            //        }
//            //    }
//            //    Rhino.RhinoApp.WriteLine("Water level at center: " + waterHeightMap[depthBitmap.Width / 2, depthBitmap.Height / 2]);

//            //    Rhino.Display.DisplayBitmap displayWaterBitmap = new Rhino.Display.DisplayBitmap(waterBitmap);
//            //    args.Display.DrawBitmap(displayWaterBitmap, 0, 0);
//            //}

//        }

//        public override BoundingBox ClippingBox
//        {
//            get
//            {
//                if (pointCloud != null)
//                {
//                    BoundingBox bbox = new BoundingBox();
//                    foreach (var point in pointCloud.GetPoints())
//                    {
//                        bbox.Union(new Point3d(point.X, point.Y, point.Z));
//                    }
//                    return bbox;
//                }
//                return BoundingBox.Empty;
//            }
//        }

//        public override void RemovedFromDocument(GH_Document document)
//        {
//            orbbecDevice?.StopCameras();
//            orbbecDevice?.Dispose();
//            base.RemovedFromDocument(document);
//        }

//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution((int)(1000.0 / 30.0), doc =>
//            {
//                ExpireSolution(false);
//            });
//        }

//        protected override Bitmap Icon => null;

//        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
//    }
//}

//////Important 
//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Windows.Forms;
//using System.Runtime.InteropServices;
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using K4AdotNet.Sensor;
//using System.IO;
//using Rhino.Runtime;
//using System.Numerics;
//using Rhino.Display;

//namespace ARSandbox
//{
//    public class ARSandboxComponent : GH_Component
//    {
//        private Device orbbecDevice = null;
//        private PointCloud pointCloud = null;
//        private Bitmap depthBitmap = null;
//        private bool isSensorConnected = false;
//        private double fx = 322, fy = 323, cx = 280, cy = 160;
//        private double minHeight = 0.85;
//        private double maxHeight = 1;
//        private double[,] waterHeightMap;

//        // Use a fixed-size array for raindrops to avoid memory allocation
//        private Raindrop[] raindrops = new Raindrop[100];
//        private int activeRaindrops = 0;

//        public ARSandboxComponent() : base("ARSandboxComponent", "OrbbecSandbox", "To view Orbbec depth on Rhino", "ARSandbox", "Subcategory") { }

//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start", "S", "Start Orbbec sensor", GH_ParamAccess.item, false);
//            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
//            pManager.AddNumberParameter("MinHeight", "MinH", "Minimum depth height (in mm)", GH_ParamAccess.item, 0.8);
//            pManager.AddNumberParameter("MaxHeight", "MaxH", "Maximum depth height (in mm)", GH_ParamAccess.item, 1);
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
//            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
//            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
//            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image", GH_ParamAccess.item);
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            double refreshRate = 30.0;
//            double newMinHeight = minHeight;
//            double newMaxHeight = maxHeight;

//            DA.GetData(0, ref startSensor);
//            DA.GetData(1, ref refreshRate);
//            DA.GetData(2, ref newMinHeight);
//            DA.GetData(3, ref newMaxHeight);

//            if (newMinHeight != minHeight || newMaxHeight != maxHeight)
//            {
//                minHeight = newMinHeight;
//                maxHeight = newMaxHeight;
//            }

//            if (startSensor && !isSensorConnected)
//            {
//                StartOrbbecFemto();
//            }

//            if (isSensorConnected && orbbecDevice != null)
//            {
//                CapturePointCloud();
//                ApplyRaindropImpacts();
//                UpdateWaterFlow();

//                // Convert PointCloud to Mesh
//                Mesh pointCloudMesh = ConvertPointCloudToMesh(pointCloud);

//                // Output PointCloud, Mesh, Depth Image
//                DA.SetDataList(0, ConvertToRhinoPoints(pointCloud));  // Points
//                DA.SetData(1, pointCloudMesh);                        // Mesh
//                DA.SetData(3, depthBitmap);                            // Depth Image
//            }

//            ScheduleSolve();
//        }

//        private void GenerateRain()
//        {
//            Random rnd = new Random();
//            for (int i = activeRaindrops; i < raindrops.Length; i++)
//            {
//                var x = rnd.Next(0, depthBitmap.Width);
//                var y = rnd.Next(0, depthBitmap.Height);
//                raindrops[i] = new Raindrop(new Point3d(x, y, maxHeight), 0.05, 0.1);
//                activeRaindrops++;
//            }
//        }

//        private void InitializeWaterHeightMap(int width, int height)
//        {
//            if (waterHeightMap == null || waterHeightMap.GetLength(0) != width || waterHeightMap.GetLength(1) != height)
//            {
//                waterHeightMap = new double[width, height];
//            }
//        }

//        private void UpdateWaterFlow()
//        {
//            // Implement a more efficient water flow algorithm here
//            // Consider using a cellular automata approach or a simplified fluid simulation
//        }

//        private void StartOrbbecFemto()
//        {
//            try
//            {
//                orbbecDevice = Device.Open();
//                if (orbbecDevice == null)
//                {
//                    Rhino.RhinoApp.WriteLine("Device not initialized.");
//                    return;
//                }

//                var config = new DeviceConfiguration
//                {
//                    DepthMode = K4AdotNet.Sensor.DepthMode.NarrowViewUnbinned,
//                    ColorResolution = ColorResolution.R1080p,
//                    CameraFps = FrameRate.Fifteen,
//                };
//                orbbecDevice.StartCameras(config);

//                isSensorConnected = true;
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
//                isSensorConnected = false;
//            }
//        }

//        private void CapturePointCloud()
//        {
//            using (var capture = orbbecDevice.GetCapture())
//            {
//                if (capture == null) return;

//                var depthImage = capture.DepthImage;
//                if (depthImage == null) return;

//                int width = depthImage.WidthPixels;
//                int height = depthImage.HeightPixels;

//                InitializeWaterHeightMap(width, height);

//                pointCloud = new PointCloud();
//                byte[] pixelBuffer = new byte[width * height * 3];

//                unsafe
//                {
//                    short* depthData = (short*)depthImage.Buffer.ToPointer();

//                    for (int y = 0; y < height; y++)
//                    {
//                        for (int x = 0; x < width; x++)
//                        {
//                            int index = y * width + x;
//                            ushort depthValue = (ushort)depthData[index];

//                            if (depthValue > 0)
//                            {
//                                double depthInMeters = depthValue * 0.001;

//                                if (depthInMeters >= minHeight && depthInMeters <= maxHeight)
//                                {
//                                    Color color = MapDepthToColor(depthInMeters);
//                                    var point = TransformDepthToPoint(x, y, depthValue);
//                                    pointCloud.Add(point, color);

//                                    pixelBuffer[index * 3 + 0] = color.B;
//                                    pixelBuffer[index * 3 + 1] = color.G;
//                                    pixelBuffer[index * 3 + 2] = color.R;

//                                    double waterLevelDepthLimit = 0.9;
//                                    if (depthInMeters > waterLevelDepthLimit)
//                                    {
//                                        waterHeightMap[x, y] = depthInMeters;
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }

//                UpdateDepthBitmap(width, height, pixelBuffer);
//            }
//        }

//        private void UpdateDepthBitmap(int width, int height, byte[] pixelBuffer)
//        {
//            if (depthBitmap == null || depthBitmap.Width != width || depthBitmap.Height != height)
//            {
//                depthBitmap?.Dispose();
//                depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//            }

//            BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//            Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//            depthBitmap.UnlockBits(bitmapData);
//        }

//        private void ApplyRaindropImpacts()
//        {
//            for (int i = 0; i < activeRaindrops; i++)
//            {
//                var raindrop = raindrops[i];
//                if (raindrop.Position.Z <= minHeight)
//                {
//                    int x = (int)raindrop.Position.X;
//                    int y = (int)raindrop.Position.Y;
//                    if (x >= 0 && y >= 0 && x < waterHeightMap.GetLength(0) && y < waterHeightMap.GetLength(1))
//                    {
//                        waterHeightMap[x, y] += raindrop.ImpactRadius;
//                    }
//                    raindrops[i] = raindrops[--activeRaindrops];
//                    i--;
//                }
//                else
//                {
//                    raindrop.Update();
//                }
//            }

//            if (activeRaindrops < raindrops.Length / 2)
//            {
//                GenerateRain();
//            }
//        }

//        private Color MapDepthToColor(double depthInMeters)
//        {
//            double normalizedDepth = (depthInMeters - minHeight) / (maxHeight - minHeight);
//            normalizedDepth = Math.Max(0, Math.Min(1, normalizedDepth));

//            int r = (int)(normalizedDepth * 255);
//            int g = (int)((1.0 - Math.Abs(normalizedDepth - 0.5) * 2) * 255);
//            int b = (int)((1.0 - normalizedDepth) * 255);

//            return Color.FromArgb(r, g, b);
//        }

//        private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//        {
//            double z = depth * 0.001;
//            double pointX = (x - cx) * z / fx;
//            double pointY = (y - cy) * z / fy;
//            return new Point3d(pointX, pointY, z);
//        }

//        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
//        {
//            List<Point3d> rhinoPoints = new List<Point3d>(pointCloud.Count);
//            foreach (var point in pointCloud)
//            {
//                rhinoPoints.Add(new Point3d(point.Location));
//            }
//            return rhinoPoints;
//        }

//        private List<Curve> GenerateContours(double[,] heightMap, double contourInterval)
//        {
//            List<Curve> contours = new List<Curve>();
//            int width = heightMap.GetLength(0);
//            int height = heightMap.GetLength(1);

//            // Iterate through the height map to find contour lines
//            for (double level = minHeight; level <= maxHeight; level += contourInterval)
//            {
//                List<Point3d> contourPoints = new List<Point3d>();

//                // Iterate through pixels in the height map and find points that match the contour level
//                for (int y = 0; y < height; y++)
//                {
//                    for (int x = 0; x < width; x++)
//                    {
//                        if (heightMap[x, y] >= level - contourInterval && heightMap[x, y] < level)
//                        {
//                            double z = heightMap[x, y];
//                            contourPoints.Add(new Point3d(x, y, z));
//                        }
//                    }
//                }

//                if (contourPoints.Count > 1)
//                {
//                    // Create a polyline or curve from the contour points
//                    Polyline contourPolyline = new Polyline(contourPoints);
//                    contours.Add(contourPolyline.ToNurbsCurve());
//                }
//            }

//            return contours;
//        }




//        public override void DrawViewportWires(IGH_PreviewArgs args)
//        {
//            if (pointCloud != null)
//            {
//                args.Display.DrawPointCloud(pointCloud, 3);
//            }
//            // Draw the Mesh from PointCloud
//            //if (pointCloud != null)
//            //{
//            //    Mesh pointCloudMesh = ConvertPointCloudToMesh(pointCloud);
//            //    // Create a display material and set it to use the vertex colors
//            //    // Ensure that the mesh has the same vertex colors as the point cloud
//            //    // Ensure that the mesh has the same vertex colors as the point cloud
//            //    if (pointCloudMesh.Vertices.Count == pointCloud.Count)
//            //    {
//            //        for (int i = 0; i < pointCloud.Count; i++)
//            //        {
//            //            var point = pointCloud[i];
//            //            var color = point.Color;  // Color from PointCloud

//            //            // Assign the color to each vertex of the mesh
//            //            pointCloudMesh.VertexColors.Add(color);
//            //        }
//            //    }

//            //    // Create a DisplayMaterial and set its Diffuse property
//            //    var material = new Rhino.Display.DisplayMaterial();
//            //    material.Diffuse = System.Drawing.Color.White; // You can change this to any desired color

//            //    // Draw the Mesh with the assigned material
//            //    //args.Display.DrawMeshShaded(pointCloudMesh, material);

//            //    //args.Display.DrawMeshWires(pointCloudMesh, System.Drawing.Color.Red);  // Default color for wires
//            //    //args.Display.DrawMeshShaded(pointCloudMesh);


//            //}






//            if (depthBitmap != null)
//            {
//                try
//                {
//                    var viewportSize = args.Viewport.Size;
//                    using (var scaledBitmap = new Bitmap(viewportSize.Width, viewportSize.Height))
//                    using (var graphics = Graphics.FromImage(scaledBitmap))
//                    {
//                        graphics.Clear(Color.Black);
//                        graphics.DrawImage(depthBitmap, 0, 0, viewportSize.Width, viewportSize.Height);
//                        args.Display.DrawBitmap(new Rhino.Display.DisplayBitmap(scaledBitmap), 0, 0);
//                    }
//                }
//                catch (ExternalException ex)
//                {
//                    Console.WriteLine($"Error creating DisplayBitmap: {ex.Message}");
//                }
//            }
//        }

//        public override BoundingBox ClippingBox
//        {
//            get
//            {
//                if (pointCloud != null && pointCloud.Count > 0)
//                {
//                    return pointCloud.GetBoundingBox(true);
//                }
//                return BoundingBox.Empty;
//            }
//        }

//        public override void RemovedFromDocument(GH_Document document)
//        {
//            orbbecDevice?.StopCameras();
//            orbbecDevice?.Dispose();
//            depthBitmap?.Dispose();
//            base.RemovedFromDocument(document);
//        }

//        // Convert PointCloud to Mesh and display in Rhino viewport
//        private Mesh ConvertPointCloudToMesh(PointCloud pointCloud)
//        {
//            // Create a new mesh to store the points and faces
//            Mesh mesh = new Mesh();

//            // Add vertices from the PointCloud to the Mesh
//            foreach (var point in pointCloud)
//            {
//                mesh.Vertices.Add(point.Location);
//                mesh.VertexColors.Add(point.Color);
//            }

//            // Generate faces using Delaunay triangulation or another method
//            // In this example, we will use a simple grid-based triangulation for illustration purposes

//            int width = (int)Math.Sqrt(pointCloud.Count);  // Assuming a square grid (for simplicity)
//            for (int y = 0; y < width - 1; y++)
//            {
//                for (int x = 0; x < width - 1; x++)
//                {
//                    // Get the indices of the four points in a 2x2 grid
//                    int i0 = y * width + x;
//                    int i1 = y * width + (x + 1);
//                    int i2 = (y + 1) * width + x;
//                    int i3 = (y + 1) * width + (x + 1);

//                    // Create two faces for each 2x2 grid
//                    mesh.Faces.AddFace(i0, i1, i2);
//                    mesh.Faces.AddFace(i1, i3, i2);
//                }
//            }

//            // You can optimize or refine the triangulation based on the actual point distribution
//            return mesh;
//        }

//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution(33, doc => ExpireSolution(false));
//        }

//        protected override Bitmap Icon => null;
//        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
//    }

//    public struct Raindrop
//    {
//        public Point3d Position;
//        public double Speed;
//        public double ImpactRadius;

//        public Raindrop(Point3d initialPosition, double speed, double impactRadius)
//        {
//            Position = initialPosition;
//            Speed = speed;
//            ImpactRadius = impactRadius;
//        }

//        public void Update()
//        {
//            Position = new Point3d(Position.X, Position.Y, Position.Z - Speed);
//        }
//    }
//}



// tried
//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Windows.Forms;
//using System.Runtime.InteropServices;
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using K4AdotNet.Sensor;
//using System.IO;
//using Rhino.Runtime;
//using System.Numerics;
//using MIConvexHull;
//using Rhino.Geometry;
//using DelaunatorSharp;
//using Rhino.Geometry;
//using System.Collections.Generic;
//using System.Linq;
//using System.Threading.Tasks;



//namespace ARSandbox
//{
//    public class ARSandboxComponent : GH_Component
//    {
//        private Device orbbecDevice = null;
//        private PointCloud pointCloud = null;
//        private Bitmap depthBitmap = null;
//        private bool isSensorConnected = false;
//        private double fx = 322, fy = 323, cx = 280, cy = 160;
//        private double minHeight = 0.85;
//        private double maxHeight = 1;
//        private double[,] waterHeightMap;

//        private Raindrop[] raindrops = new Raindrop[100];
//        private int activeRaindrops = 0;

//        public ARSandboxComponent() : base("ARSandboxComponent", "OrbbecSandbox", "To view Orbbec depth on Rhino", "ARSandbox", "Subcategory") { }

//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start", "S", "Start Orbbec sensor", GH_ParamAccess.item, false);
//            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
//            pManager.AddNumberParameter("MinHeight", "MinH", "Minimum depth height (in mm)", GH_ParamAccess.item, 0.8);
//            pManager.AddNumberParameter("MaxHeight", "MaxH", "Maximum depth height (in mm)", GH_ParamAccess.item, 1);
//            pManager.AddBooleanParameter("Save Surface", "Save", "Save the generated surface to a file", GH_ParamAccess.item, false);
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
//            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
//            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
//            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image", GH_ParamAccess.item);
//            pManager.AddBrepParameter("Surface", "S", "Generated surface from point cloud", GH_ParamAccess.item); // Added output for surface
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            double refreshRate = 30.0;
//            double newMinHeight = minHeight;
//            double newMaxHeight = maxHeight;
//            bool saveSurface = false; // New input

//            DA.GetData(0, ref startSensor);
//            DA.GetData(1, ref refreshRate);
//            DA.GetData(2, ref newMinHeight);
//            DA.GetData(3, ref newMaxHeight);
//            DA.GetData(4, ref saveSurface); // Get "Save Surface" boolean

//            if (newMinHeight != minHeight || newMaxHeight != maxHeight)
//            {
//                minHeight = newMinHeight;
//                maxHeight = newMaxHeight;
//            }

//            if (startSensor && !isSensorConnected)
//            {
//                StartOrbbecFemto();
//            }

//            if (isSensorConnected && orbbecDevice != null)
//            {
//                CapturePointCloud();
//                //ApplyRaindropImpacts();
//                UpdateWaterFlow();

//                DA.SetDataList(0, ConvertToRhinoPoints(pointCloud));
//                DA.SetData(3, depthBitmap);

//                // Generate surface from the point cloud
//                var surface = CreateSurfaceFromPointCloud(pointCloud);
//                DA.SetData(4, surface); // Set the generated surface
//                                        // Save the surface if requested
//                if (saveSurface && surface != null)
//                {
//                    string savePath = "D:\\Ghplugin\\ARSandbox\\surfaces\\GeneratedSurface.3dm";
//                    SaveSurfaceToFile(surface, savePath);
//                }
//            }


//            ScheduleSolve();
//        }

//        private void SaveSurfaceToFile(Brep surface, string filePath)
//        {
//            if (surface == null)
//            {
//                Rhino.RhinoApp.WriteLine("No surface to save.");
//                return;
//            }

//            try
//            {
//                // Create a document to store the Brep
//                var doc = new Rhino.FileIO.File3dm();

//                // Add the surface to the document
//                doc.Objects.AddBrep(surface);

//                // Save the document to the specified file path
//                if (doc.Write(filePath, 6)) // Use Rhino version 6 or compatible
//                {
//                    Rhino.RhinoApp.WriteLine($"Surface saved successfully to {filePath}");
//                }
//                else
//                {
//                    Rhino.RhinoApp.WriteLine("Failed to save the surface.");
//                }

//                doc.Dispose();
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine($"Error saving surface: {ex.Message}");
//            }
//        }



//        private Brep CreateSurfaceFromPointCloud(PointCloud cloud)
//        {
//            if (cloud == null || cloud.Count < 3)
//                return null;

//            // Extract points from the point cloud
//            var points = new List<Point3d>();
//            foreach (var pt in cloud)
//            {
//                points.Add(pt.Location);
//            }

//            // Use fully qualified Rhino.Geometry.Plane to avoid ambiguity
//            Rhino.Geometry.Plane plane;
//            Rhino.Geometry.Plane.FitPlaneToPoints(points, out plane);

//            var projectedPoints = new List<Point2d>();
//            foreach (var pt in points)
//            {
//                double u, v;
//                plane.ClosestParameter(pt, out u, out v);  // Fix: Provide both u and v out parameters
//                projectedPoints.Add(new Point2d(u, v));
//            }

//            // Perform 2D Delaunay triangulation
//            var delaunayMesh = new Mesh();
//            try
//            {
//                delaunayMesh = DelaunayTriangulation2D(points, plane);
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine($"Error during triangulation: {ex.Message}");
//            }

//            if (delaunayMesh == null || delaunayMesh.Vertices.Count < 3)
//                return null;

//            // Convert the mesh to a Brep (surface)
//            return Brep.CreateFromMesh(delaunayMesh, true);
//        }


//        private Mesh DelaunayTriangulation2D(List<Point3d> originalPoints, Rhino.Geometry.Plane plane)
//        {
//            // Project 3D points onto the plane (convert them into 2D points)
//            var projectedPoints = new List<Point2d>();
//            foreach (var pt in originalPoints)
//            {
//                double u, v;
//                plane.ClosestParameter(pt, out u, out v);  // Project 3D point to 2D coordinates
//                projectedPoints.Add(new Point2d(u, v));
//            }

//            // Convert the projected points into a list of IPoint (DelaunatorSharp.IPoint)
//            var flatPoints = projectedPoints.Select(p => new DelaunatorSharp.Point { X = p.X, Y = p.Y }).Cast<IPoint>().ToArray();

//            // Perform Delaunay triangulation using Delaunator
//            var delaunator = new Delaunator(flatPoints);
//            var triangles = delaunator.Triangles;  // Triangles is a list of indices into the original points

//            // Create a Rhino Mesh from the triangulation
//            var finalMesh = new Mesh();

//            foreach (var triangle in triangles)
//            {
//                // Access the 2D points in the triangle using the indices
//                var ptA = plane.PointAt(flatPoints[triangle].X, flatPoints[triangle].Y);
//                var ptB = plane.PointAt(flatPoints[triangle].X, flatPoints[triangle].Y);
//                var ptC = plane.PointAt(flatPoints[triangle].X, flatPoints[triangle].Y);

//                // Add the points to the mesh
//                finalMesh.Vertices.Add(ptA);
//                finalMesh.Vertices.Add(ptB);
//                finalMesh.Vertices.Add(ptC);

//                // Create the mesh faces (triangle)
//                finalMesh.Faces.AddFace(finalMesh.Vertices.Count - 3, finalMesh.Vertices.Count - 2, finalMesh.Vertices.Count - 1);
//            }

//            finalMesh.Normals.ComputeNormals();
//            finalMesh.Compact();

//            return finalMesh;
//        }

//        private void GenerateRain()
//        {
//            Random rnd = new Random();
//            for (int i = activeRaindrops; i < raindrops.Length; i++)
//            {
//                var x = rnd.Next(0, depthBitmap.Width);
//                var y = rnd.Next(0, depthBitmap.Height);
//                raindrops[i] = new Raindrop(new Point3d(x, y, maxHeight), 0.05, 0.1);
//                activeRaindrops++;
//            }
//        }

//        private void InitializeWaterHeightMap(int width, int height)
//        {
//            if (waterHeightMap == null || waterHeightMap.GetLength(0) != width || waterHeightMap.GetLength(1) != height)
//            {
//                waterHeightMap = new double[width, height];
//            }
//        }

//        private void UpdateWaterFlow()
//        {
//            // Implement a more efficient water flow algorithm here
//        }

//        private void StartOrbbecFemto()
//        {
//            try
//            {
//                orbbecDevice = Device.Open();
//                if (orbbecDevice == null)
//                {
//                    Rhino.RhinoApp.WriteLine("Device not initialized.");
//                    return;
//                }

//                var config = new DeviceConfiguration
//                {
//                    DepthMode = DepthMode.NarrowViewUnbinned,
//                    ColorResolution = ColorResolution.R1080p,
//                    CameraFps = FrameRate.Fifteen,
//                };
//                orbbecDevice.StartCameras(config);

//                isSensorConnected = true;
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
//                isSensorConnected = false;
//            }
//        }

//        private void CapturePointCloud()
//        {
//            using (var capture = orbbecDevice.GetCapture())
//            {
//                if (capture == null) return;

//                var depthImage = capture.DepthImage;
//                if (depthImage == null) return;

//                int width = depthImage.WidthPixels;
//                int height = depthImage.HeightPixels;

//                InitializeWaterHeightMap(width, height);
//                pointCloud = new PointCloud();

//                byte[] pixelBuffer = new byte[width * height * 3];
//                Parallel.For(0, height, y =>
//                {
//                    unsafe
//                    {
//                        short* depthData = (short*)depthImage.Buffer.ToPointer();
//                        for (int x = 0; x < width; x++)
//                        {
//                            int index = y * width + x;
//                            ushort depthValue = (ushort)depthData[index];

//                            if (depthValue > 0)
//                            {
//                                double depthInMeters = depthValue * 0.001;
//                                if (depthInMeters >= minHeight && depthInMeters <= maxHeight)
//                                {
//                                    Color color = MapDepthToColor(depthInMeters);
//                                    var point = TransformDepthToPoint(x, y, depthValue);
//                                    lock (pointCloud)
//                                    {
//                                        pointCloud.Add(point, color);
//                                    }

//                                    int pixelIndex = index * 3;
//                                    pixelBuffer[pixelIndex] = color.B;
//                                    pixelBuffer[pixelIndex + 1] = color.G;
//                                    pixelBuffer[pixelIndex + 2] = color.R;

//                                    lock (waterHeightMap)
//                                    {
//                                        waterHeightMap[x, y] = depthInMeters;
//                                    }
//                                }
//                            }
//                        }
//                    }
//                });

//                UpdateDepthBitmap(width, height, pixelBuffer);
//            }
//        }


//        private void UpdateDepthBitmap(int width, int height, byte[] pixelBuffer)
//        {
//            if (depthBitmap == null || depthBitmap.Width != width || depthBitmap.Height != height)
//            {
//                depthBitmap?.Dispose();
//                depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//            }

//            BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//            Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//            depthBitmap.UnlockBits(bitmapData);
//        }

//        private Color MapDepthToColor(double depthInMeters)
//        {
//            double normalizedDepth = (depthInMeters - minHeight) / (maxHeight - minHeight);
//            normalizedDepth = Math.Max(0, Math.Min(1, normalizedDepth));

//            int r = (int)(normalizedDepth * 255);
//            int g = (int)((1.0 - Math.Abs(normalizedDepth - 0.5) * 2) * 255);
//            int b = (int)((1.0 - normalizedDepth) * 255);

//            return Color.FromArgb(r, g, b);
//        }

//        private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//        {
//            double z = depth * 0.001;
//            double pointX = (x - cx) * z / fx;
//            double pointY = (y - cy) * z / fy;
//            return new Point3d(pointX, pointY, z);
//        }

//        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
//        {
//            List<Point3d> rhinoPoints = new List<Point3d>(pointCloud.Count);
//            foreach (var point in pointCloud)
//            {
//                rhinoPoints.Add(new Point3d(point.Location));
//            }
//            return rhinoPoints;
//        }

//        public override void DrawViewportWires(IGH_PreviewArgs args)
//        {
//            if (pointCloud != null)
//            {
//                args.Display.DrawPointCloud(pointCloud, 3);
//            }

//            //if (depthBitmap != null)
//            //{
//            //    try
//            //    {
//            //        var viewportSize = args.Viewport.Size;
//            //        using (var scaledBitmap = new Bitmap(viewportSize.Width, viewportSize.Height))
//            //        using (var graphics = Graphics.FromImage(scaledBitmap))
//            //        {
//            //            graphics.Clear(Color.Black);
//            //            graphics.DrawImage(depthBitmap, new Rectangle(0, 0, viewportSize.Width, viewportSize.Height));
//            //            using (var displayBitmap = new Rhino.Display.DisplayBitmap(scaledBitmap))
//            //            {
//            //                args.Display.DrawBitmap(displayBitmap, 0, 0);
//            //            }
//            //        }
//            //    }
//            //    catch (ExternalException ex)
//            //    {
//            //        Console.WriteLine("Error creating DisplayBitmap: " + ex.Message);
//            //    }
//            //}
//        }

//        public override BoundingBox ClippingBox
//        {
//            get
//            {
//                if (pointCloud != null && pointCloud.Count > 0)
//                {
//                    return pointCloud.GetBoundingBox(true);
//                }
//                return BoundingBox.Empty;
//            }
//        }

//        public override void RemovedFromDocument(GH_Document document)
//        {
//            orbbecDevice?.StopCameras();
//            orbbecDevice?.Dispose();
//            depthBitmap?.Dispose();
//            base.RemovedFromDocument(document);
//        }

//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution(33, doc => ExpireSolution(false));
//        }

//        protected override Bitmap Icon => null;
//        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
//    }

//    public struct Raindrop
//    {
//        public Point3d Position;
//        public double Speed;
//        public double ImpactRadius;

//        public Raindrop(Point3d initialPosition, double speed, double impactRadius)
//        {
//            Position = initialPosition;
//            Speed = speed;
//            ImpactRadius = impactRadius;
//        }

//        public void Update()
//        {
//            Position = new Point3d(Position.X, Position.Y, Position.Z - Speed);
//        }
//    }
//}




//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Windows.Forms;
//using System.Runtime.InteropServices;
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using K4AdotNet.Sensor;
//using System.IO;
//using Rhino.Runtime;
//using System.Numerics;
//using Rhino.Display;

//namespace ARSandbox
//{
//    public class ARSandboxComponent : GH_Component
//    {
//        private Device orbbecDevice = null;
//        private PointCloud pointCloud = null;
//        private Bitmap depthBitmap = null;
//        private bool isSensorConnected = false;
//        private double fx = 322, fy = 323, cx = 280, cy = 160;
//        private double minHeight = 0.85;
//        private double maxHeight = 1;
//        private double[,] waterHeightMap;

//        // Use a fixed-size array for raindrops to avoid memory allocation
//        private Raindrop[] raindrops = new Raindrop[100];
//        private int activeRaindrops = 0;

//        public ARSandboxComponent() : base("ARSandboxComponent", "OrbbecSandbox", "To view Orbbec depth on Rhino", "ARSandbox", "Subcategory") { }

//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start", "S", "Start Orbbec sensor", GH_ParamAccess.item, false);
//            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
//            pManager.AddNumberParameter("MinHeight", "MinH", "Minimum depth height (in mm)", GH_ParamAccess.item, 0.8);
//            pManager.AddNumberParameter("MaxHeight", "MaxH", "Maximum depth height (in mm)", GH_ParamAccess.item, 1);
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
//            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
//            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
//            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image", GH_ParamAccess.item);
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            double refreshRate = 30.0;
//            double newMinHeight = minHeight;
//            double newMaxHeight = maxHeight;

//            DA.GetData(0, ref startSensor);
//            DA.GetData(1, ref refreshRate);
//            DA.GetData(2, ref newMinHeight);
//            DA.GetData(3, ref newMaxHeight);

//            if (newMinHeight != minHeight || newMaxHeight != maxHeight)
//            {
//                minHeight = newMinHeight;
//                maxHeight = newMaxHeight;
//            }

//            if (startSensor && !isSensorConnected)
//            {
//                StartOrbbecFemto();
//            }

//            if (isSensorConnected && orbbecDevice != null)
//            {
//                CapturePointCloud();
//                ApplyRaindropImpacts();
//                UpdateWaterFlow();

//                // Convert PointCloud to Mesh
//                Mesh pointCloudMesh = ConvertPointCloudToMesh(pointCloud);
//                // Generate contours
//                double contourInterval = 0.05; // You can adjust this value
//                List<Curve> contours = GenerateContours(waterHeightMap, contourInterval);
//                // Output PointCloud, Mesh, Depth Image
//                DA.SetDataList(0, ConvertToRhinoPoints(pointCloud));  // Points
//                DA.SetData(1, pointCloudMesh);
//                DA.SetDataList(2, contours); // Contours// Mesh
//                DA.SetData(3, depthBitmap);                            // Depth Image
//                                                                       // Log contour information
//                LogContourInfo(contours);
//            }

//            ScheduleSolve();
//        }
//        private void LogContourInfo(List<Curve> contours)
//        {
//            string logFilePath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Desktop), "ContourLog.txt");
//            using (StreamWriter writer = new StreamWriter(logFilePath, true))
//            {
//                writer.WriteLine($"Contour Log - {DateTime.Now}");
//                writer.WriteLine($"Total Contours: {contours.Count}");

//                for (int i = 0; i < contours.Count; i++)
//                {
//                    Curve contour = contours[i];
//                    writer.WriteLine($"Contour {i + 1}:");
//                    writer.WriteLine($"  Length: {contour.GetLength():F2}");
//                    writer.WriteLine($"  Start Point: {contour.PointAtStart}");
//                    writer.WriteLine($"  End Point: {contour.PointAtEnd}");
//                    writer.WriteLine();
//                }

//                writer.WriteLine("----------------------------------------");
//            }
//        }

//        private void GenerateRain()
//        {
//            Random rnd = new Random();
//            for (int i = activeRaindrops; i < raindrops.Length; i++)
//            {
//                var x = rnd.Next(0, depthBitmap.Width);
//                var y = rnd.Next(0, depthBitmap.Height);
//                raindrops[i] = new Raindrop(new Point3d(x, y, maxHeight), 0.05, 0.1);
//                activeRaindrops++;
//            }
//        }

//        private void InitializeWaterHeightMap(int width, int height)
//        {
//            if (waterHeightMap == null || waterHeightMap.GetLength(0) != width || waterHeightMap.GetLength(1) != height)
//            {
//                waterHeightMap = new double[width, height];
//            }
//        }

//        private void UpdateWaterFlow()
//        {
//            // Implement a more efficient water flow algorithm here
//            // Consider using a cellular automata approach or a simplified fluid simulation
//        }

//        private void StartOrbbecFemto()
//        {
//            try
//            {
//                orbbecDevice = Device.Open();
//                if (orbbecDevice == null)
//                {
//                    Rhino.RhinoApp.WriteLine("Device not initialized.");
//                    return;
//                }

//                var config = new DeviceConfiguration
//                {
//                    DepthMode = K4AdotNet.Sensor.DepthMode.NarrowViewUnbinned,
//                    ColorResolution = ColorResolution.R1080p,
//                    CameraFps = FrameRate.Fifteen,
//                };
//                orbbecDevice.StartCameras(config);

//                isSensorConnected = true;
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
//                isSensorConnected = false;
//            }
//        }

//        private void CapturePointCloud()
//        {
//            using (var capture = orbbecDevice.GetCapture())
//            {
//                if (capture == null) return;

//                var depthImage = capture.DepthImage;
//                if (depthImage == null) return;

//                int width = depthImage.WidthPixels;
//                int height = depthImage.HeightPixels;

//                InitializeWaterHeightMap(width, height);

//                pointCloud = new PointCloud();
//                byte[] pixelBuffer = new byte[width * height * 3];

//                unsafe
//                {
//                    short* depthData = (short*)depthImage.Buffer.ToPointer();

//                    for (int y = 0; y < height; y++)
//                    {
//                        for (int x = 0; x < width; x++)
//                        {
//                            int index = y * width + x;
//                            ushort depthValue = (ushort)depthData[index];

//                            if (depthValue > 0)
//                            {
//                                double depthInMeters = depthValue * 0.001;

//                                if (depthInMeters >= minHeight && depthInMeters <= maxHeight)
//                                {
//                                    Color color = MapDepthToColor(depthInMeters);
//                                    var point = TransformDepthToPoint(x, y, depthValue);
//                                    pointCloud.Add(point, color);

//                                    pixelBuffer[index * 3 + 0] = color.B;
//                                    pixelBuffer[index * 3 + 1] = color.G;
//                                    pixelBuffer[index * 3 + 2] = color.R;

//                                    double waterLevelDepthLimit = 0.9;
//                                    if (depthInMeters > waterLevelDepthLimit)
//                                    {
//                                        waterHeightMap[x, y] = depthInMeters;
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }

//                UpdateDepthBitmap(width, height, pixelBuffer);
//            }
//        }

//        private void UpdateDepthBitmap(int width, int height, byte[] pixelBuffer)
//        {
//            if (depthBitmap == null || depthBitmap.Width != width || depthBitmap.Height != height)
//            {
//                depthBitmap?.Dispose();
//                depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//            }

//            BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//            Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//            depthBitmap.UnlockBits(bitmapData);
//        }

//        private void ApplyRaindropImpacts()
//        {
//            for (int i = 0; i < activeRaindrops; i++)
//            {
//                var raindrop = raindrops[i];
//                if (raindrop.Position.Z <= minHeight)
//                {
//                    int x = (int)raindrop.Position.X;
//                    int y = (int)raindrop.Position.Y;
//                    if (x >= 0 && y >= 0 && x < waterHeightMap.GetLength(0) && y < waterHeightMap.GetLength(1))
//                    {
//                        waterHeightMap[x, y] += raindrop.ImpactRadius;
//                    }
//                    raindrops[i] = raindrops[--activeRaindrops];
//                    i--;
//                }
//                else
//                {
//                    raindrop.Update();
//                }
//            }

//            if (activeRaindrops < raindrops.Length / 2)
//            {
//                GenerateRain();
//            }
//        }

//        private Color MapDepthToColor(double depthInMeters)
//        {
//            double normalizedDepth = (depthInMeters - minHeight) / (maxHeight - minHeight);
//            normalizedDepth = Math.Max(0, Math.Min(1, normalizedDepth));

//            int r = (int)(normalizedDepth * 255);
//            int g = (int)((1.0 - Math.Abs(normalizedDepth - 0.5) * 2) * 255);
//            int b = (int)((1.0 - normalizedDepth) * 255);

//            return Color.FromArgb(r, g, b);
//        }

//        private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//        {
//            double z = depth * 0.001;
//            double pointX = (x - cx) * z / fx;
//            double pointY = (y - cy) * z / fy;
//            return new Point3d(pointX, pointY, z);
//        }

//        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
//        {
//            List<Point3d> rhinoPoints = new List<Point3d>(pointCloud.Count);
//            foreach (var point in pointCloud)
//            {
//                rhinoPoints.Add(new Point3d(point.Location));
//            }
//            return rhinoPoints;
//        }

//        private List<Curve> GenerateContours(double[,] heightMap, double contourInterval)
//        {
//            List<Curve> contours = new List<Curve>();
//            int width = heightMap.GetLength(0);
//            int height = heightMap.GetLength(1);

//            // Iterate through the height map to find contour lines
//            for (double level = minHeight; level <= maxHeight; level += contourInterval)
//            {
//                List<Point3d> contourPoints = new List<Point3d>();

//                // Iterate through pixels in the height map and find points that match the contour level
//                for (int y = 0; y < height; y++)
//                {
//                    for (int x = 0; x < width; x++)
//                    {
//                        if (heightMap[x, y] >= level - contourInterval && heightMap[x, y] < level)
//                        {
//                            double z = heightMap[x, y];
//                            contourPoints.Add(new Point3d(x, y, z));
//                        }
//                    }
//                }

//                if (contourPoints.Count > 1)
//                {
//                    // Create a polyline or curve from the contour points
//                    Polyline contourPolyline = new Polyline(contourPoints);
//                    contours.Add(contourPolyline.ToNurbsCurve());
//                }
//            }

//            return contours;
//        }




//        public override void DrawViewportWires(IGH_PreviewArgs args)
//        {
//            if (pointCloud != null)
//            {
//                args.Display.DrawPointCloud(pointCloud, 3);
//            }
//            // Draw the Mesh from PointCloud
//            //if (pointCloud != null)
//            //{
//            //    Mesh pointCloudMesh = ConvertPointCloudToMesh(pointCloud);
//            //    // Create a display material and set it to use the vertex colors
//            //    // Ensure that the mesh has the same vertex colors as the point cloud
//            //    // Ensure that the mesh has the same vertex colors as the point cloud
//            //    if (pointCloudMesh.Vertices.Count == pointCloud.Count)
//            //    {
//            //        for (int i = 0; i < pointCloud.Count; i++)
//            //        {
//            //            var point = pointCloud[i];
//            //            var color = point.Color;  // Color from PointCloud

//            //            // Assign the color to each vertex of the mesh
//            //            pointCloudMesh.VertexColors.Add(color);
//            //        }
//            //    }

//            //    // Create a DisplayMaterial and set its Diffuse property
//            //    var material = new Rhino.Display.DisplayMaterial();
//            //    material.Diffuse = System.Drawing.Color.White; // You can change this to any desired color

//            //    // Draw the Mesh with the assigned material
//            //    //args.Display.DrawMeshShaded(pointCloudMesh, material);

//            //    //args.Display.DrawMeshWires(pointCloudMesh, System.Drawing.Color.Red);  // Default color for wires
//            //    //args.Display.DrawMeshShaded(pointCloudMesh);


//            //}

//            // Draw contours
//            if (waterHeightMap != null)
//            {
//                double contourInterval = 0.05; // You can adjust this value
//                List<Curve> contours = GenerateContours(waterHeightMap, contourInterval);

//                foreach (Curve contour in contours)
//                {
//                    args.Display.DrawCurve(contour, System.Drawing.Color.Blue, 2);
//                }
//            }




//            //if (depthBitmap != null)
//            //{
//            //    try
//            //    {
//            //        var viewportSize = args.Viewport.Size;
//            //        using (var scaledBitmap = new Bitmap(viewportSize.Width, viewportSize.Height))
//            //        using (var graphics = Graphics.FromImage(scaledBitmap))
//            //        {
//            //            graphics.Clear(Color.Black);
//            //            graphics.DrawImage(depthBitmap, 0, 0, viewportSize.Width, viewportSize.Height);
//            //            args.Display.DrawBitmap(new Rhino.Display.DisplayBitmap(scaledBitmap), 0, 0);
//            //        }
//            //    }
//            //    catch (ExternalException ex)
//            //    {
//            //        Console.WriteLine($"Error creating DisplayBitmap: {ex.Message}");
//            //    }
//            //}
//        }

//        public override BoundingBox ClippingBox
//        {
//            get
//            {
//                if (pointCloud != null && pointCloud.Count > 0)
//                {
//                    return pointCloud.GetBoundingBox(true);
//                }
//                return BoundingBox.Empty;
//            }
//        }

//        public override void RemovedFromDocument(GH_Document document)
//        {
//            orbbecDevice?.StopCameras();
//            orbbecDevice?.Dispose();
//            depthBitmap?.Dispose();
//            base.RemovedFromDocument(document);
//        }

//        // Convert PointCloud to Mesh and display in Rhino viewport
//        private Mesh ConvertPointCloudToMesh(PointCloud pointCloud)
//        {
//            // Create a new mesh to store the points and faces
//            Mesh mesh = new Mesh();

//            // Add vertices from the PointCloud to the Mesh
//            foreach (var point in pointCloud)
//            {
//                mesh.Vertices.Add(point.Location);
//                mesh.VertexColors.Add(point.Color);
//            }

//            // Generate faces using Delaunay triangulation or another method
//            // In this example, we will use a simple grid-based triangulation for illustration purposes

//            int width = (int)Math.Sqrt(pointCloud.Count);  // Assuming a square grid (for simplicity)
//            for (int y = 0; y < width - 1; y++)
//            {
//                for (int x = 0; x < width - 1; x++)
//                {
//                    // Get the indices of the four points in a 2x2 grid
//                    int i0 = y * width + x;
//                    int i1 = y * width + (x + 1);
//                    int i2 = (y + 1) * width + x;
//                    int i3 = (y + 1) * width + (x + 1);

//                    // Create two faces for each 2x2 grid
//                    mesh.Faces.AddFace(i0, i1, i2);
//                    mesh.Faces.AddFace(i1, i3, i2);
//                }
//            }

//            // You can optimize or refine the triangulation based on the actual point distribution
//            return mesh;
//        }

//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution(33, doc => ExpireSolution(false));
//        }

//        protected override Bitmap Icon => null;
//        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
//    }

//    public struct Raindrop
//    {
//        public Point3d Position;
//        public double Speed;
//        public double ImpactRadius;

//        public Raindrop(Point3d initialPosition, double speed, double impactRadius)
//        {
//            Position = initialPosition;
//            Speed = speed;
//            ImpactRadius = impactRadius;
//        }

//        public void Update()
//        {
//            Position = new Point3d(Position.X, Position.Y, Position.Z - Speed);
//        }
//    }
//}

////removed raindrop and contours
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using Grasshopper.Kernel;
using Rhino.Geometry;
using K4AdotNet.Sensor;
using Rhino.Display;

namespace ARSandbox
{
    public class ARSandboxComponent : GH_Component
    {
        private Device orbbecDevice = null;
        private PointCloud pointCloud = null;
        private Bitmap depthBitmap = null;
        private bool isSensorConnected = false;
        private double fx = 322, fy = 323, cx = 280, cy = 160;
        private double minHeight = 0.85;
        private double maxHeight = 1;

        public ARSandboxComponent() : base("ARSandboxComponent", "OrbbecSandbox", "To view Orbbec depth on Rhino", "ARSandbox", "Subcategory") { }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Start", "S", "Start Orbbec sensor", GH_ParamAccess.item, false);
            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
            pManager.AddNumberParameter("MinHeight", "MinH", "Minimum depth height (in mm)", GH_ParamAccess.item, 0.8);
            pManager.AddNumberParameter("MaxHeight", "MaxH", "Maximum depth height (in mm)", GH_ParamAccess.item, 1);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool startSensor = false;
            double refreshRate = 30.0;
            double newMinHeight = minHeight;
            double newMaxHeight = maxHeight;

            DA.GetData(0, ref startSensor);
            DA.GetData(1, ref refreshRate);
            DA.GetData(2, ref newMinHeight);
            DA.GetData(3, ref newMaxHeight);

            if (newMinHeight != minHeight || newMaxHeight != maxHeight)
            {
                minHeight = newMinHeight;
                maxHeight = newMaxHeight;
            }

            if (startSensor && !isSensorConnected)
            {
                StartOrbbecFemto();
            }

            if (isSensorConnected && orbbecDevice != null)
            {
                CapturePointCloud();

                // Convert PointCloud to Mesh
                Mesh pointCloudMesh = ConvertPointCloudToMesh(pointCloud);

                // Generate contours
                double contourInterval = 0.05; // You can adjust this value
                List<Curve> contours = GenerateContours(pointCloud, contourInterval);

                // Output PointCloud, Mesh, Contours, Depth Image
                DA.SetDataList(0, ConvertToRhinoPoints(pointCloud)); // Points
                DA.SetData(1, pointCloudMesh); // Mesh
                DA.SetDataList(2, contours); // Contours
                DA.SetData(3, depthBitmap); // Depth Image
            }

            ScheduleSolve();
        }

        private void StartOrbbecFemto()
        {
            try
            {
                orbbecDevice = Device.Open();
                if (orbbecDevice == null)
                {
                    Rhino.RhinoApp.WriteLine("Device not initialized.");
                    return;
                }

                var config = new DeviceConfiguration
                {
                    DepthMode = K4AdotNet.Sensor.DepthMode.NarrowViewUnbinned,
                    ColorResolution = ColorResolution.R1080p,
                    CameraFps = FrameRate.Fifteen,
                };

                orbbecDevice.StartCameras(config);
                isSensorConnected = true;
            }
            catch (Exception ex)
            {
                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
                isSensorConnected = false;
            }
        }

        private void CapturePointCloud()
        {
            using (var capture = orbbecDevice.GetCapture())
            {
                if (capture == null) return;
                var depthImage = capture.DepthImage;
                if (depthImage == null) return;

                int width = depthImage.WidthPixels;
                int height = depthImage.HeightPixels;

                pointCloud = new PointCloud();
                byte[] pixelBuffer = new byte[width * height * 3];

                unsafe
                {
                    short* depthData = (short*)depthImage.Buffer.ToPointer();
                    for (int y = 0; y < height; y++)
                    {
                        for (int x = 0; x < width; x++)
                        {
                            int index = y * width + x;
                            ushort depthValue = (ushort)depthData[index];

                            if (depthValue > 0)
                            {
                                double depthInMeters = depthValue * 0.001;
                                if (depthInMeters >= minHeight && depthInMeters <= maxHeight)
                                {
                                    Color color = MapDepthToColor(depthInMeters);
                                    var point = TransformDepthToPoint(x, y, depthValue);
                                    pointCloud.Add(point, color);
                                    pixelBuffer[index * 3 + 0] = color.B;
                                    pixelBuffer[index * 3 + 1] = color.G;
                                    pixelBuffer[index * 3 + 2] = color.R;
                                }
                            }
                        }
                    }
                }

                UpdateDepthBitmap(width, height, pixelBuffer);
            }
        }

        private void UpdateDepthBitmap(int width, int height, byte[] pixelBuffer)
        {
            if (depthBitmap == null || depthBitmap.Width != width || depthBitmap.Height != height)
            {
                depthBitmap?.Dispose();
                depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
            }

            BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
            Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
            depthBitmap.UnlockBits(bitmapData);
        }

        private Color MapDepthToColor(double depthInMeters)
        {
            double normalizedDepth = (depthInMeters - minHeight) / (maxHeight - minHeight);
            normalizedDepth = Math.Max(0, Math.Min(1, normalizedDepth));
            int r = (int)(normalizedDepth * 255);
            int g = (int)((1.0 - Math.Abs(normalizedDepth - 0.5) * 2) * 255);
            int b = (int)((1.0 - normalizedDepth) * 255);
            return Color.FromArgb(r, g, b);
        }

        private Point3d TransformDepthToPoint(int x, int y, ushort depth)
        {
            double z = depth * 0.001;
            double pointX = (x - cx) * z / fx;
            double pointY = (y - cy) * z / fy;
            return new Point3d(pointX, pointY, z);
        }

        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
        {
            List<Point3d> rhinoPoints = new List<Point3d>(pointCloud.Count);
            foreach (var point in pointCloud)
            {
                rhinoPoints.Add(new Point3d(point.Location));
            }
            return rhinoPoints;
        }

        private List<Curve> GenerateContours(PointCloud pointCloud, double contourInterval)
        {
            List<Curve> contours = new List<Curve>();
            // Implement contour generation based on the point cloud
            // This is a simplified placeholder and should be replaced with actual contour generation logic
            return contours;
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            if (pointCloud != null)
            {
                args.Display.DrawPointCloud(pointCloud, 3);
            }

            // Draw contours
            if (pointCloud != null)
            {
                double contourInterval = 0.05; // You can adjust this value
                List<Curve> contours = GenerateContours(pointCloud, contourInterval);
                foreach (Curve contour in contours)
                {
                    args.Display.DrawCurve(contour, System.Drawing.Color.Blue, 2);
                }
            }
            //if (depthBitmap != null)
            //{
            //    try
            //    {
            //        var viewportSize = args.Viewport.Size;
            //        using (var scaledBitmap = new Bitmap(viewportSize.Width, viewportSize.Height))
            //        using (var graphics = Graphics.FromImage(scaledBitmap))
            //        {
            //            graphics.Clear(Color.Black);
            //            graphics.DrawImage(depthBitmap, 0, 0, viewportSize.Width, viewportSize.Height);
            //            args.Display.DrawBitmap(new Rhino.Display.DisplayBitmap(scaledBitmap), 0, 0);
            //        }
            //    }
            //    catch (ExternalException ex)
            //    {
            //        Console.WriteLine($"Error creating DisplayBitmap: {ex.Message}");
            //    }
            //}
        }

        public override BoundingBox ClippingBox
        {
            get
            {
                if (pointCloud != null && pointCloud.Count > 0)
                {
                    return pointCloud.GetBoundingBox(true);
                }
                return BoundingBox.Empty;
            }
        }

        public override void RemovedFromDocument(GH_Document document)
        {
            orbbecDevice?.StopCameras();
            orbbecDevice?.Dispose();
            depthBitmap?.Dispose();
            base.RemovedFromDocument(document);
        }

        private Mesh ConvertPointCloudToMesh(PointCloud pointCloud)
        {
            Mesh mesh = new Mesh();
            foreach (var point in pointCloud)
            {
                mesh.Vertices.Add(point.Location);
                mesh.VertexColors.Add(point.Color);
            }

            // Implement mesh face generation logic here
            // This is a placeholder and should be replaced with proper triangulation

            return mesh;
        }

        protected void ScheduleSolve()
        {
            OnPingDocument().ScheduleSolution(33, doc => ExpireSolution(false));
        }

        protected override Bitmap Icon => null;
        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
    }
}

///Target mesh
//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Runtime.InteropServices;
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using K4AdotNet.Sensor;
//using Rhino.Display;
//using System.Linq;
//using System.IO;

//namespace ARSandbox
//{
//    public class ARSandboxComponent : GH_Component
//    {
//        private Device orbbecDevice = null;
//        private PointCloud pointCloud = null;
//        private Bitmap depthBitmap = null;
//        private bool isSensorConnected = false;
//        private double fx = 322, fy = 323, cx = 280, cy = 160;
//        private double minHeight = 0.85;
//        private double maxHeight = 1;

//        public ARSandboxComponent() : base("ARSandboxComponent", "OrbbecSandbox", "To view Orbbec depth on Rhino", "ARSandbox", "Subcategory") { }

//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start", "S", "Start Orbbec sensor", GH_ParamAccess.item, false);
//            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
//            pManager.AddNumberParameter("MinHeight", "MinH", "Minimum depth height (in mm)", GH_ParamAccess.item, 0.8);
//            pManager.AddNumberParameter("MaxHeight", "MaxH", "Maximum depth height (in mm)", GH_ParamAccess.item, 1);
//            pManager.AddTextParameter("File Path", "F", "Path to the target mesh file", GH_ParamAccess.item, @"C:\Users\MLA\Documents\test.stl");
//            pManager.AddMeshParameter("TargetMesh", "T", "Target topography mesh for comparison", GH_ParamAccess.item);
//        }
//        private void LogMeshDetails(Mesh mesh, string filePath)
//        {
//            string logFilePath = @"D:\mesh_log.txt"; // You can change this path to any location you prefer.

//            using (StreamWriter writer = new StreamWriter(logFilePath, true))
//            {
//                writer.WriteLine($"Log for Mesh at {DateTime.Now}: {filePath}");
//                writer.WriteLine($"Vertices: {mesh.Vertices.Count}");
//                writer.WriteLine($"Faces: {mesh.Faces.Count}");
//                writer.WriteLine($"Valid: {mesh.IsValid}");
//                writer.WriteLine("--------------------------------------------------");
//            }
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
//            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
//            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
//            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image", GH_ParamAccess.item);
//            pManager.AddMeshParameter("Difference Map", "Diff", "Mesh showing areas to add/remove sand", GH_ParamAccess.item);
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            double refreshRate = 30.0;
//            string filePath = string.Empty;
//            Mesh targetMesh = null;

//            // Retrieve input data
//            if (!DA.GetData(0, ref startSensor) || !startSensor)
//            {
//                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Set 'Start' to true to initialize the sensor.");
//                return;
//            }

//            DA.GetData(1, ref refreshRate);
//            DA.GetData(2, ref minHeight);
//            DA.GetData(3, ref maxHeight);
//            DA.GetData(4, ref filePath);

//            // Retrieve the TargetMesh input (if available)
//            DA.GetData(5, ref targetMesh);

//            // Check if the target mesh is provided, if not, we can continue without it
//            if (targetMesh == null || !targetMesh.IsValid)
//            {
//                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Target mesh is not provided or is invalid. Proceeding without it.");
//            }
//            else
//            {
//                // Log the provided mesh details (if it's valid)
//                LogMeshDetails(targetMesh, "Provided Input Mesh");
//            }

//            // Start Orbbec sensor if not already connected
//            if (startSensor && !isSensorConnected)
//            {
//                StartOrbbecFemto();
//            }

//            if (isSensorConnected && orbbecDevice != null)
//            {
//                // Capture depth data and generate point cloud
//                CapturePointCloud();

//                // Convert point cloud to mesh
//                Mesh liveMesh = ConvertPointCloudToMesh(pointCloud);
//                if (liveMesh == null || !liveMesh.IsValid)
//                {
//                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Failed to generate live mesh from point cloud.");
//                    return;
//                }

//                // Generate contours
//                List<Curve> contours = GenerateContoursFromMesh(liveMesh, 0.05);

//                // If targetMesh is provided, compute the difference map, otherwise skip it
//                Mesh differenceMap = null;
//                if (targetMesh != null && targetMesh.IsValid)
//                {
//                    differenceMap = ComputeDifferenceMap(liveMesh, targetMesh);
//                }

//                // Output results
//                DA.SetDataList(0, ConvertToRhinoPoints(pointCloud));
//                DA.SetData(1, liveMesh);
//                DA.SetDataList(2, contours);
//                DA.SetData(3, depthBitmap);
//                DA.SetData(4, differenceMap);
//            }
//            else
//            {
//                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Sensor is not connected or failed to initialize.");
//            }

//            ScheduleSolve();
//        }





//        private Mesh LoadTargetMesh(string filePath)
//        {
//            Mesh targetMesh = new Mesh();
//            if (System.IO.File.Exists(filePath))
//            {
//                // Import the file using Rhino's mesh file reading capabilities
//                var meshes = Rhino.FileIO.File3dm.Read(filePath).Objects.Where(obj => obj.Geometry is Mesh)
//                                      .Select(obj => obj.Geometry as Mesh);

//                foreach (var mesh in meshes)
//                {
//                    targetMesh.Append(mesh);
//                }
//            }
//            else
//            {
//                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "File not found: " + filePath);
//            }
//            return targetMesh;
//        }


//        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
//        {
//            List<Point3d> rhinoPoints = new List<Point3d>(pointCloud.Count);
//            foreach (var point in pointCloud)
//            {
//                rhinoPoints.Add(new Point3d(point.Location));
//            }
//            return rhinoPoints;
//        }
//            private void StartOrbbecFemto()
//        {
//            try
//            {
//                orbbecDevice = Device.Open();
//                if (orbbecDevice == null)
//                {
//                    Rhino.RhinoApp.WriteLine("Device not initialized.");
//                    return;
//                }

//                var config = new DeviceConfiguration
//                {
//                    DepthMode = K4AdotNet.Sensor.DepthMode.NarrowViewUnbinned,
//                    ColorResolution = ColorResolution.R1080p,
//                    CameraFps = FrameRate.Thirty,
//                };

//                orbbecDevice.StartCameras(config);
//                isSensorConnected = true;
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
//                isSensorConnected = false;
//            }
//        }

//        private Color MapDepthToColor(double depthInMeters)
//        {
//            double normalizedDepth = (depthInMeters - minHeight) / (maxHeight - minHeight);
//            normalizedDepth = Math.Max(0, Math.Min(1, normalizedDepth));
//            int r = (int)(normalizedDepth * 255);
//            int g = (int)((1.0 - Math.Abs(normalizedDepth - 0.5) * 2) * 255);
//            int b = (int)((1.0 - normalizedDepth) * 255);
//            return Color.FromArgb(r, g, b);
//        }





//        private void CapturePointCloud()
//        {
//            using (var capture = orbbecDevice.GetCapture())
//            {
//                if (capture == null) return;
//                var depthImage = capture.DepthImage;
//                if (depthImage == null) return;

//                int width = depthImage.WidthPixels;
//                int height = depthImage.HeightPixels;

//                pointCloud = new PointCloud();
//                byte[] pixelBuffer = new byte[width * height * 3];

//                unsafe
//                {
//                    short* depthData = (short*)depthImage.Buffer.ToPointer();
//                    for (int y = 0; y < height; y++)
//                    {
//                        for (int x = 0; x < width; x++)
//                        {
//                            int index = y * width + x;
//                            ushort depthValue = (ushort)depthData[index];

//                            if (depthValue > 0)
//                            {
//                                double depthInMeters = depthValue * 0.001;
//                                if (depthInMeters >= minHeight && depthInMeters <= maxHeight)
//                                {
//                                    Color color = MapDepthToColor(depthInMeters);
//                                    var point = TransformDepthToPoint(x, y, depthValue);
//                                    pointCloud.Add(point, color);
//                                    pixelBuffer[index * 3 + 0] = color.B;
//                                    pixelBuffer[index * 3 + 1] = color.G;
//                                    pixelBuffer[index * 3 + 2] = color.R;
//                                }
//                            }
//                        }
//                    }
//                }

//                UpdateDepthBitmap(width, height, pixelBuffer);
//            }
//        }

//        private Mesh ConvertPointCloudToMesh(PointCloud pointCloud)
//        {
//            Mesh mesh = new Mesh();

//            // Collect all the points from the point cloud
//            List<Point3d> points = new List<Point3d>();
//            foreach (var point in pointCloud)
//            {
//                points.Add(point.Location);
//            }

//            // Add the points to the mesh
//            foreach (var point in points)
//            {
//                mesh.Vertices.Add(point);
//            }

//            // Assuming points are ordered in a way that makes sense for triangulation
//            // You can manually add faces based on some logic (like a grid or Delaunay, if using an external library)

//            // Example: creating a simple grid or manual face creation (simplified for demonstration)
//            for (int i = 0; i < points.Count - 2; i++)
//            {
//                if (i + 1 < points.Count && i + 2 < points.Count)
//                {
//                    mesh.Faces.AddFace(i, i + 1, i + 2);
//                }
//            }

//            // This approach assumes that your points are in order and you have a method of defining the faces.
//            return mesh;
//        }



//        private List<Curve> GenerateContoursFromMesh(Mesh mesh, double interval)
//        {
//            List<Curve> contours = new List<Curve>();
//            Plane plane = Plane.WorldXY;

//            for (double z = minHeight; z <= maxHeight; z += interval)
//            {
//                plane.OriginZ = z;

//                // Get the intersection polylines
//                var sections = Rhino.Geometry.Intersect.Intersection.MeshPlane(mesh, plane);
//                if (sections != null)
//                {
//                    foreach (var polyline in sections)
//                    {
//                        if (polyline != null && polyline.IsValid)
//                        {
//                            // Convert Polyline to NurbsCurve (Curve)
//                            Curve curve = polyline.ToNurbsCurve();
//                            contours.Add(curve);
//                        }
//                    }
//                }
//            }

//            return contours;
//        }

//        private Mesh ComputeDifferenceMap(Mesh liveMesh, Mesh targetMesh)
//        {
//            Mesh differenceMesh = new Mesh();

//            // Calculate vertex differences
//            foreach (var liveVertex in liveMesh.Vertices)
//            {
//                Point3d livePoint = new Point3d(liveVertex.X, liveVertex.Y, liveVertex.Z);
//                double closestDistance = double.MaxValue;
//                Point3d closestTargetPoint = Point3d.Unset;

//                foreach (var targetVertex in targetMesh.Vertices)
//                {
//                    Point3d targetPoint = new Point3d(targetVertex.X, targetVertex.Y, targetVertex.Z);
//                    double distance = livePoint.DistanceTo(targetPoint);

//                    if (distance < closestDistance)
//                    {
//                        closestDistance = distance;
//                        closestTargetPoint = targetPoint;
//                    }
//                }

//                double heightDifference = livePoint.Z - closestTargetPoint.Z;
//                Color color = MapHeightDifferenceToColor(heightDifference);
//                differenceMesh.Vertices.Add(livePoint);
//                differenceMesh.VertexColors.Add(color);
//            }

//            return differenceMesh;
//        }

//        private Color MapHeightDifferenceToColor(double heightDifference)
//        {
//            if (heightDifference < -0.1)
//                return Color.Red; // Remove sand
//            if (heightDifference > 0.1)
//                return Color.Blue; // Add sand
//            return Color.Green; // Neutral
//        }

//        private void UpdateDepthBitmap(int width, int height, byte[] pixelBuffer)
//        {
//            if (depthBitmap == null || depthBitmap.Width != width || depthBitmap.Height != height)
//            {
//                depthBitmap?.Dispose();
//                depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//            }

//            BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//            Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//            depthBitmap.UnlockBits(bitmapData);
//        }

//        private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//        {
//            double z = depth * 0.001;
//            double pointX = (x - cx) * z / fx;
//            double pointY = (y - cy) * z / fy;
//            return new Point3d(pointX, pointY, z);
//        }
//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution(33, doc => ExpireSolution(false));
//        }

//        protected override Bitmap Icon => null;
//        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
//    }
//}



//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Runtime.InteropServices;
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using K4AdotNet.Sensor;
//using Rhino.Display;
//using System.Linq;
//using System.IO;
//using System.Threading.Tasks;

//namespace ARSandbox
//{
//    public class ARSandboxComponent : GH_Component
//    {
//        private Device orbbecDevice = null;
//        private PointCloud pointCloud = null;
//        private Bitmap depthBitmap = null;
//        private bool isSensorConnected = false;
//        private double fx = 322, fy = 323, cx = 280, cy = 160;
//        private double minHeight = 0.85;
//        private double maxHeight = 1;

//        public ARSandboxComponent() : base("ARSandboxComponent", "OrbbecSandbox", "To view Orbbec depth on Rhino", "ARSandbox", "Subcategory") { }

//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start", "S", "Start Orbbec sensor", GH_ParamAccess.item, false);
//            pManager.AddNumberParameter("RefreshRate", "R", "Point cloud refresh rate", GH_ParamAccess.item, 30.0);
//            pManager.AddNumberParameter("MinHeight", "MinH", "Minimum depth height (in mm)", GH_ParamAccess.item, 0.8);
//            pManager.AddNumberParameter("MaxHeight", "MaxH", "Maximum depth height (in mm)", GH_ParamAccess.item, 1);
//            pManager.AddTextParameter("File Path", "F", "Path to the target mesh file", GH_ParamAccess.item, @"C:\Users\MLA\Documents\test.stl");
//            pManager.AddMeshParameter("TargetMesh", "T", "Target topography mesh for comparison", GH_ParamAccess.item);
//        }
//        private void LogMeshDetails(Mesh mesh, string filePath)
//        {
//            string logFilePath = @"D:\mesh_log.txt"; // You can change this path to any location you prefer.

//            using (StreamWriter writer = new StreamWriter(logFilePath, true))
//            {
//                writer.WriteLine($"Log for Mesh at {DateTime.Now}: {filePath}");
//                writer.WriteLine($"Vertices: {mesh.Vertices.Count}");
//                writer.WriteLine($"Faces: {mesh.Faces.Count}");
//                writer.WriteLine($"Valid: {mesh.IsValid}");
//                writer.WriteLine("--------------------------------------------------");
//            }
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddPointParameter("Points", "P", "Generated point cloud", GH_ParamAccess.list);
//            pManager.AddMeshParameter("Mesh", "M", "Generated mesh from point cloud", GH_ParamAccess.item);
//            pManager.AddCurveParameter("Contours", "C", "Generated contours", GH_ParamAccess.list);
//            pManager.AddGenericParameter("Depth Image", "D", "Captured depth image", GH_ParamAccess.item);
//            pManager.AddMeshParameter("Difference Map", "Diff", "Mesh showing areas to add/remove sand", GH_ParamAccess.item);
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            double refreshRate = 30.0;
//            string filePath = string.Empty;
//            Mesh targetMesh = null;

//            // Retrieve input data
//            if (!DA.GetData(0, ref startSensor) || !startSensor)
//            {
//                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Set 'Start' to true to initialize the sensor.");
//                return;
//            }

//            DA.GetData(1, ref refreshRate);
//            DA.GetData(2, ref minHeight);
//            DA.GetData(3, ref maxHeight);
//            DA.GetData(4, ref filePath);

//            // Retrieve the TargetMesh input (if available)
//            DA.GetData(5, ref targetMesh);

//            if (targetMesh == null || !targetMesh.IsValid)
//            {
//                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Target mesh is not provided or is invalid. Proceeding without it.");
//            }
//            else
//            {
//                // Log the provided mesh details (if it's valid)
//                LogMeshDetails(targetMesh, "Provided Input Mesh");
//            }

//            // Start Orbbec sensor in a background thread if not already connected
//            if (startSensor && !isSensorConnected)
//            {
//                Task.Run(() => StartOrbbecFemto());
//                return; // Wait for the sensor to initialize asynchronously
//            }

//            if (isSensorConnected && orbbecDevice != null)
//            {
//                // Process the point cloud and other operations in a separate thread
//                Task.Run(() =>
//                {
//                    try
//                    {
//                        CapturePointCloud();

//                        // Convert point cloud to mesh
//                        Mesh liveMesh = ConvertPointCloudToMesh(pointCloud);
//                        if (liveMesh == null || !liveMesh.IsValid)
//                        {
//                            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Failed to generate live mesh from point cloud.");
//                            return;
//                        }

//                        // Generate contours
//                        List<Curve> contours = GenerateContoursFromMesh(liveMesh, 0.05);

//                        // Compute difference map if target mesh is provided
//                        Mesh differenceMap = null;
//                        if (targetMesh != null && targetMesh.IsValid)
//                        {
//                            differenceMap = ComputeDifferenceMap(liveMesh, targetMesh);
//                        }

//                        // Output results (use a thread-safe context for Grasshopper updates)
//                        Rhino.RhinoApp.InvokeOnUiThread(new Action(() =>
//                        {
//                            DA.SetDataList(0, ConvertToRhinoPoints(pointCloud));
//                            DA.SetData(1, liveMesh);
//                            DA.SetDataList(2, contours);
//                            DA.SetData(3, depthBitmap);
//                            DA.SetData(4, differenceMap);
//                        }));

//                    }
//                    catch (Exception ex)
//                    {
//                        Rhino.RhinoApp.WriteLine("Error during processing: " + ex.Message);
//                    }
//                });
//            }
//            else
//            {
//                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Sensor is not connected or failed to initialize.");
//            }

//            // Schedule solve only when necessary
//            if (isSensorConnected)
//            {
//                ScheduleSolve();
//            }
//        }







//        private Mesh LoadTargetMesh(string filePath)
//        {
//            Mesh targetMesh = new Mesh();
//            if (System.IO.File.Exists(filePath))
//            {
//                // Import the file using Rhino's mesh file reading capabilities
//                var meshes = Rhino.FileIO.File3dm.Read(filePath).Objects.Where(obj => obj.Geometry is Mesh)
//                                      .Select(obj => obj.Geometry as Mesh);

//                foreach (var mesh in meshes)
//                {
//                    targetMesh.Append(mesh);
//                }
//            }
//            else
//            {
//                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "File not found: " + filePath);
//            }
//            return targetMesh;
//        }


//        private List<Point3d> ConvertToRhinoPoints(PointCloud pointCloud)
//        {
//            List<Point3d> rhinoPoints = new List<Point3d>(pointCloud.Count);
//            foreach (var point in pointCloud)
//            {
//                rhinoPoints.Add(new Point3d(point.Location));
//            }
//            return rhinoPoints;
//        }
//        private void StartOrbbecFemto()
//        {
//            try
//            {
//                orbbecDevice = Device.Open();
//                if (orbbecDevice == null)
//                {
//                    Rhino.RhinoApp.WriteLine("Device not initialized.");
//                    return;
//                }

//                var config = new DeviceConfiguration
//                {
//                    DepthMode = K4AdotNet.Sensor.DepthMode.NarrowViewUnbinned,
//                    ColorResolution = ColorResolution.R1080p,
//                    CameraFps = FrameRate.Thirty,
//                };

//                orbbecDevice.StartCameras(config);
//                isSensorConnected = true;
//            }
//            catch (Exception ex)
//            {
//                Rhino.RhinoApp.WriteLine("Error connecting to Orbbec Femto: " + ex.Message);
//                isSensorConnected = false;
//            }
//        }

//        private Color MapDepthToColor(double depthInMeters)
//        {
//            double normalizedDepth = (depthInMeters - minHeight) / (maxHeight - minHeight);
//            normalizedDepth = Math.Max(0, Math.Min(1, normalizedDepth));
//            int r = (int)(normalizedDepth * 255);
//            int g = (int)((1.0 - Math.Abs(normalizedDepth - 0.5) * 2) * 255);
//            int b = (int)((1.0 - normalizedDepth) * 255);
//            return Color.FromArgb(r, g, b);
//        }





//        private void CapturePointCloud()
//        {
//            using (var capture = orbbecDevice.GetCapture())
//            {
//                if (capture == null) return;
//                var depthImage = capture.DepthImage;
//                if (depthImage == null) return;

//                int width = depthImage.WidthPixels;
//                int height = depthImage.HeightPixels;

//                pointCloud = new PointCloud();
//                byte[] pixelBuffer = new byte[width * height * 3];

//                unsafe
//                {
//                    short* depthData = (short*)depthImage.Buffer.ToPointer();
//                    for (int y = 0; y < height; y++)
//                    {
//                        for (int x = 0; x < width; x++)
//                        {
//                            int index = y * width + x;
//                            ushort depthValue = (ushort)depthData[index];

//                            if (depthValue > 0)
//                            {
//                                double depthInMeters = depthValue * 0.001;
//                                if (depthInMeters >= minHeight && depthInMeters <= maxHeight)
//                                {
//                                    Color color = MapDepthToColor(depthInMeters);
//                                    var point = TransformDepthToPoint(x, y, depthValue);
//                                    pointCloud.Add(point, color);
//                                    pixelBuffer[index * 3 + 0] = color.B;
//                                    pixelBuffer[index * 3 + 1] = color.G;
//                                    pixelBuffer[index * 3 + 2] = color.R;
//                                }
//                            }
//                        }
//                    }
//                }

//                UpdateDepthBitmap(width, height, pixelBuffer);
//            }
//        }

//        private Mesh ConvertPointCloudToMesh(PointCloud pointCloud)
//        {
//            Mesh mesh = new Mesh();

//            // Collect all the points from the point cloud
//            List<Point3d> points = new List<Point3d>();
//            foreach (var point in pointCloud)
//            {
//                points.Add(point.Location);
//            }

//            // Add the points to the mesh
//            foreach (var point in points)
//            {
//                mesh.Vertices.Add(point);
//            }

//            // Assuming points are ordered in a way that makes sense for triangulation
//            // You can manually add faces based on some logic (like a grid or Delaunay, if using an external library)

//            // Example: creating a simple grid or manual face creation (simplified for demonstration)
//            for (int i = 0; i < points.Count - 2; i++)
//            {
//                if (i + 1 < points.Count && i + 2 < points.Count)
//                {
//                    mesh.Faces.AddFace(i, i + 1, i + 2);
//                }
//            }

//            // This approach assumes that your points are in order and you have a method of defining the faces.
//            return mesh;
//        }



//        private List<Curve> GenerateContoursFromMesh(Mesh mesh, double interval)
//        {
//            List<Curve> contours = new List<Curve>();
//            Plane plane = Plane.WorldXY;

//            for (double z = minHeight; z <= maxHeight; z += interval)
//            {
//                plane.OriginZ = z;

//                // Get the intersection polylines
//                var sections = Rhino.Geometry.Intersect.Intersection.MeshPlane(mesh, plane);
//                if (sections != null)
//                {
//                    foreach (var polyline in sections)
//                    {
//                        if (polyline != null && polyline.IsValid)
//                        {
//                            // Convert Polyline to NurbsCurve (Curve)
//                            Curve curve = polyline.ToNurbsCurve();
//                            contours.Add(curve);
//                        }
//                    }
//                }
//            }

//            return contours;
//        }

//        private Mesh ComputeDifferenceMap(Mesh liveMesh, Mesh targetMesh)
//        {
//            Mesh differenceMesh = new Mesh();

//            // Calculate vertex differences
//            foreach (var liveVertex in liveMesh.Vertices)
//            {
//                Point3d livePoint = new Point3d(liveVertex.X, liveVertex.Y, liveVertex.Z);
//                double closestDistance = double.MaxValue;
//                Point3d closestTargetPoint = Point3d.Unset;

//                foreach (var targetVertex in targetMesh.Vertices)
//                {
//                    Point3d targetPoint = new Point3d(targetVertex.X, targetVertex.Y, targetVertex.Z);
//                    double distance = livePoint.DistanceTo(targetPoint);

//                    if (distance < closestDistance)
//                    {
//                        closestDistance = distance;
//                        closestTargetPoint = targetPoint;
//                    }
//                }

//                double heightDifference = livePoint.Z - closestTargetPoint.Z;
//                Color color = MapHeightDifferenceToColor(heightDifference);
//                differenceMesh.Vertices.Add(livePoint);
//                differenceMesh.VertexColors.Add(color);
//            }

//            return differenceMesh;
//        }

//        private Color MapHeightDifferenceToColor(double heightDifference)
//        {
//            if (heightDifference < -0.1)
//                return Color.Red; // Remove sand
//            if (heightDifference > 0.1)
//                return Color.Blue; // Add sand
//            return Color.Green; // Neutral
//        }

//        private void UpdateDepthBitmap(int width, int height, byte[] pixelBuffer)
//        {
//            if (depthBitmap == null || depthBitmap.Width != width || depthBitmap.Height != height)
//            {
//                depthBitmap?.Dispose();
//                depthBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
//            }

//            BitmapData bitmapData = depthBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, depthBitmap.PixelFormat);
//            Marshal.Copy(pixelBuffer, 0, bitmapData.Scan0, pixelBuffer.Length);
//            depthBitmap.UnlockBits(bitmapData);
//        }

//        private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//        {
//            double z = depth * 0.001;
//            double pointX = (x - cx) * z / fx;
//            double pointY = (y - cy) * z / fy;
//            return new Point3d(pointX, pointY, z);
//        }
//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution(33, doc => ExpireSolution(false));
//        }

//        protected override Bitmap Icon => null;
//        public override Guid ComponentGuid => new Guid("18f05d10-9a3f-46e4-863e-42ecb7c3cd45");
//    }
//}



//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Drawing.Imaging;
//using System.Runtime.InteropServices;
//using Grasshopper.Kernel;
//using Rhino.Geometry;
//using Rhino.Display;
//using K4AdotNet.Sensor;

//namespace ARSandbox
//{
//    public class ComputeDifferenceMapComponent : GH_Component
//    {
//        private Device depthSensor = null;
//        private Bitmap depthBitmap = null;
//        private PointCloud currentPointCloud = null;
//        private bool isSensorActive = false;
//        private double fx = 322, fy = 323, cx = 280, cy = 160;
//        private double minHeight = 0.85;
//        private double maxHeight = 1.0;

//        public ComputeDifferenceMapComponent()
//            : base("Compute Difference Map", "DiffMap",
//                   "Computes difference between depth map and target mesh.",
//                   "ARSandbox", "Analysis")
//        { }

//        protected override void RegisterInputParams(GH_InputParamManager pManager)
//        {
//            pManager.AddBooleanParameter("Start", "S", "Start depth sensor", GH_ParamAccess.item, false);
//            pManager.AddMeshParameter("TargetMesh", "TM", "Mesh to compare against", GH_ParamAccess.item);
//            pManager.AddNumberParameter("MinHeight", "MinH", "Minimum height (in meters)", GH_ParamAccess.item, 0.8);
//            pManager.AddNumberParameter("MaxHeight", "MaxH", "Maximum height (in meters)", GH_ParamAccess.item, 1.0);
//        }

//        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
//        {
//            pManager.AddGenericParameter("Difference Map", "DM", "Bitmap difference map", GH_ParamAccess.item);
//            pManager.AddPointParameter("Points Above", "PA", "Points above the target mesh", GH_ParamAccess.list);
//            pManager.AddPointParameter("Points Below", "PB", "Points below the target mesh", GH_ParamAccess.list);
//        }

//        protected override void SolveInstance(IGH_DataAccess DA)
//        {
//            bool startSensor = false;
//            Mesh targetMesh = null;

//            DA.GetData(0, ref startSensor);
//            DA.GetData(1, ref targetMesh);
//            DA.GetData(2, ref minHeight);
//            DA.GetData(3, ref maxHeight);

//            if (startSensor && !isSensorActive)
//            {
//                StartSensor();
//            }

//            if (isSensorActive && depthSensor != null && targetMesh != null)
//            {
//                CapturePointCloud();

//                Compute difference between depth and target mesh
//               Bitmap differenceMap;
//                List<Point3d> pointsAbove, pointsBelow;
//                ComputeDifference(targetMesh, out differenceMap, out pointsAbove, out pointsBelow);

//                Output results
//                DA.SetData(0, differenceMap);    // Difference Map
//                DA.SetDataList(1, pointsAbove); // Points Above
//                DA.SetDataList(2, pointsBelow); // Points Below
//            }

//            ScheduleSolve();
//        }

//        private void StartSensor()
//        {
//            try
//            {
//                depthSensor = Device.Open();
//                var config = new DeviceConfiguration
//                {
//                    DepthMode = K4AdotNet.Sensor.DepthMode.NarrowViewUnbinned,
//                    ColorResolution = ColorResolution.R1080p,
//                    CameraFps = FrameRate.Fifteen,
//                };
//                depthSensor.StartCameras(config);
//                isSensorActive = true;
//            }
//            catch (Exception ex)
//            {
//                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Sensor Error: {ex.Message}");
//                isSensorActive = false;
//            }
//        }

//        private void CapturePointCloud()
//        {
//            using (var capture = depthSensor.GetCapture())
//            {
//                if (capture == null) return;
//                var depthImage = capture.DepthImage;
//                if (depthImage == null) return;

//                int width = depthImage.WidthPixels;
//                int height = depthImage.HeightPixels;
//                currentPointCloud = new PointCloud();
//                byte[] pixelBuffer = new byte[width * height * 3];

//                unsafe
//                {
//                    short* depthData = (short*)depthImage.Buffer.ToPointer();
//                    for (int y = 0; y < height; y++)
//                    {
//                        for (int x = 0; x < width; x++)
//                        {
//                            int index = y * width + x;
//                            ushort depthValue = (ushort)depthData[index];

//                            if (depthValue > 0)
//                            {
//                                double depthInMeters = depthValue * 0.001;
//                                if (depthInMeters >= minHeight && depthInMeters <= maxHeight)
//                                {
//                                    var point = TransformDepthToPoint(x, y, depthValue);
//                                    currentPointCloud.Add(point, Color.White);
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }

//        private void ComputeDifference(Mesh targetMesh, out Bitmap differenceMap, out List<Point3d> pointsAbove, out List<Point3d> pointsBelow)
//        {
//            differenceMap = new Bitmap(640, 480, PixelFormat.Format24bppRgb); // Adjust resolution as needed
//            pointsAbove = new List<Point3d>();
//            pointsBelow = new List<Point3d>();

//            BitmapData bitmapData = differenceMap.LockBits(new Rectangle(0, 0, 640, 480), ImageLockMode.WriteOnly, differenceMap.PixelFormat);

//            foreach (var point in currentPointCloud)
//            {
//                Point3d testPoint = point.Location;
//                double meshHeight = targetMesh.ClosestPoint(testPoint).Z;

//                if (testPoint.Z > meshHeight)
//                {
//                    pointsAbove.Add(testPoint);
//                    DrawPointToBitmap(bitmapData, testPoint, Color.Red); // Above
//                }
//                else if (testPoint.Z < meshHeight)
//                {
//                    pointsBelow.Add(testPoint);
//                    DrawPointToBitmap(bitmapData, testPoint, Color.Blue); // Below
//                }
//            }

//            differenceMap.UnlockBits(bitmapData);
//        }

//        private void DrawPointToBitmap(BitmapData bitmapData, Point3d point, Color color)
//        {
//            int x = (int)((point.X * fx / point.Z) + cx);
//            int y = (int)((point.Y * fy / point.Z) + cy);
//            if (x >= 0 && x < bitmapData.Width && y >= 0 && y < bitmapData.Height)
//            {
//                unsafe
//                {
//                    byte* pixelPtr = (byte*)bitmapData.Scan0 + (y * bitmapData.Stride) + (x * 3);
//                    pixelPtr[0] = color.B;
//                    pixelPtr[1] = color.G;
//                    pixelPtr[2] = color.R;
//                }
//            }
//        }

//        private Point3d TransformDepthToPoint(int x, int y, ushort depth)
//        {
//            double z = depth * 0.001;
//            double pointX = (x - cx) * z / fx;
//            double pointY = (y - cy) * z / fy;
//            return new Point3d(pointX, pointY, z);
//        }

//        protected void ScheduleSolve()
//        {
//            OnPingDocument().ScheduleSolution(33, doc => ExpireSolution(false));
//        }

//        protected override Bitmap Icon => null;
//        public override Guid ComponentGuid => new Guid("92b50f17-2f93-4295-bb91-2b4fddcd1ad1");
//    }
//}
