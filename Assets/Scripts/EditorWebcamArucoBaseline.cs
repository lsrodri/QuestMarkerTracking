using System;
using System.Collections.Generic;
using UnityEngine;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.UnityIntegration;
using OpenCVForUnity.ObjdetectModule;
using OpenCVForUnity.Calib3dModule;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.UnityIntegration.Helper.Source2Mat;
using static OpenCVForUnity.UnityIntegration.OpenCVARUtils;

namespace TryAR.MarkerTracking
{
    /// <summary>
    /// Phase 1: Editor Baseline using OpenCV for Unity and WebCamTexture
    /// This script acts as a ground-truth reference for ArUco marker detection
    /// before migrating to the native C++ plugin.
    /// </summary>
    public class EditorWebcamArucoBaseline : MonoBehaviour
    {
        [Header("Webcam Settings")]
        [SerializeField] private string requestedDeviceName = "";
        [SerializeField] private int requestedWidth = 1280;
        [SerializeField] private int requestedHeight = 720;
        [SerializeField] private int requestedFPS = 30;

        [Header("Marker Settings")]
        [SerializeField] private int targetMarkerId = 0;
        [SerializeField] private GameObject targetObject;
        [SerializeField] private float markerLength = 0.1f;
        [SerializeField] private ArUcoMarkerTracking.ArUcoDictionary dictionaryId = ArUcoMarkerTracking.ArUcoDictionary.DICT_4X4_50;

        [Header("Camera Parameters")]
        [Tooltip("If true, estimates FOV from Unity Camera. Otherwise uses a default pinhole matrix.")]
        [SerializeField] private bool useUnityCameraIntrinsics = true;
        [SerializeField] private Camera mainCamera;

        private WebCamTexture webCamTexture;
        private Texture2D m_cameraTexture;
        private Color32[] colors;

        // OpenCV mats
        private Mat rgbaMat;
        private Mat rgbMat;
        private Mat cameraIntrinsicMatrix;
        private MatOfDouble cameraDistortionCoeffs;
        
        // Detection
        private Dictionary markerDictionary;
        private ArucoDetector arucoDetector;
        private Mat detectedMarkerIds;
        private List<Mat> detectedMarkerCorners;
        private List<Mat> rejectedMarkerCandidates;

        private bool isReady = false;

        void Start()
        {
            if (mainCamera == null)
                mainCamera = Camera.main;

            InitializeWebcam();
        }

        private void InitializeWebcam()
        {
            if (WebCamTexture.devices.Length == 0)
            {
                Debug.LogError("No webcam found.");
                return;
            }

            string deviceName = requestedDeviceName;
            if (string.IsNullOrEmpty(deviceName))
                deviceName = WebCamTexture.devices[0].name;

            webCamTexture = new WebCamTexture(deviceName, requestedWidth, requestedHeight, requestedFPS);
            webCamTexture.Play();
        }

        void Update()
        {
            if (!webCamTexture.isPlaying || webCamTexture.didUpdateThisFrame == false)
                return;

            if (webCamTexture.width > 16 && webCamTexture.height > 16)
            {
                if (!isReady)
                {
                    InitializeOpenCV(webCamTexture.width, webCamTexture.height);
                    isReady = true;
                }

                ProcessFrame();
            }
        }

        private void InitializeOpenCV(int width, int height)
        {
            rgbaMat = new Mat(height, width, CvType.CV_8UC4);
            rgbMat = new Mat(height, width, CvType.CV_8UC3);
            m_cameraTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);
            colors = new Color32[width * height];

            // Setup camera intrinsics
            cameraIntrinsicMatrix = new Mat(3, 3, CvType.CV_64FC1);
            float fx, fy, cx, cy;
            
            if (useUnityCameraIntrinsics && mainCamera != null)
            {
                // Approximate from Unity camera FOV
                float fovY = mainCamera.fieldOfView;
                float fovX = 2.0f * Mathf.Atan(Mathf.Tan(fovY * Mathf.Deg2Rad * 0.5f) * mainCamera.aspect) * Mathf.Rad2Deg;
                fx = (width / 2.0f) / Mathf.Tan(fovX * Mathf.Deg2Rad * 0.5f);
                fy = (height / 2.0f) / Mathf.Tan(fovY * Mathf.Deg2Rad * 0.5f);
                cx = width / 2.0f;
                cy = height / 2.0f;
            }
            else
            {
                // Default pinhole camera
                fx = width; 
                fy = width;
                cx = width / 2.0f;
                cy = height / 2.0f;
            }

            cameraIntrinsicMatrix.put(0, 0, fx);
            cameraIntrinsicMatrix.put(0, 1, 0);
            cameraIntrinsicMatrix.put(0, 2, cx);
            cameraIntrinsicMatrix.put(1, 0, 0);
            cameraIntrinsicMatrix.put(1, 1, fy);
            cameraIntrinsicMatrix.put(1, 2, cy);
            cameraIntrinsicMatrix.put(2, 0, 0);
            cameraIntrinsicMatrix.put(2, 1, 0);
            cameraIntrinsicMatrix.put(2, 2, 1.0f);

            cameraDistortionCoeffs = new MatOfDouble(0, 0, 0, 0);

            // Aruco setup
            markerDictionary = Objdetect.getPredefinedDictionary((int)dictionaryId);
            DetectorParameters detectorParams = new DetectorParameters();
            detectorParams.set_useAruco3Detection(true);
            detectorParams.set_cornerRefinementMethod(Objdetect.CORNER_REFINE_SUBPIX);
            RefineParameters refineParameters = new RefineParameters(10f, 3f, true);
            arucoDetector = new ArucoDetector(markerDictionary, detectorParams, refineParameters);

            detectedMarkerIds = new Mat();
            detectedMarkerCorners = new List<Mat>();
            rejectedMarkerCandidates = new List<Mat>();
        }

        private void ProcessFrame()
        {
            // Read webcam texture to OpenCV Mat
            OpenCVMatUtils.WebCamTextureToMat(webCamTexture, rgbaMat, colors);
            Imgproc.cvtColor(rgbaMat, rgbMat, Imgproc.COLOR_RGBA2RGB);

            detectedMarkerIds.create(0, 1, CvType.CV_32S);
            detectedMarkerCorners.Clear();
            rejectedMarkerCandidates.Clear();

            arucoDetector.detectMarkers(rgbMat, detectedMarkerCorners, detectedMarkerIds, rejectedMarkerCandidates);

            // Visualize (optional, if we were displaying it, but here we just do pose estimation)
            if (detectedMarkerCorners.Count > 0 && targetObject != null)
            {
                EstimatePose();
                Debug.Log("Marker detected! Marker ID: " + detectedMarkerIds.get(0, 0)[0]);
            }
        }

        private void EstimatePose()
        {
            using (MatOfPoint3f objectPoints = new MatOfPoint3f(
                new Point3(-markerLength / 2f, markerLength / 2f, 0),
                new Point3(markerLength / 2f, markerLength / 2f, 0),
                new Point3(markerLength / 2f, -markerLength / 2f, 0),
                new Point3(-markerLength / 2f, -markerLength / 2f, 0)
            ))
            {
                for (int i = 0; i < detectedMarkerCorners.Count; i++)
                {
                    int currentId = (int)detectedMarkerIds.get(i, 0)[0];
                    if (currentId != targetMarkerId)
                        continue;

                    using (Mat rotationVec = new Mat(1, 1, CvType.CV_64FC3))
                    using (Mat translationVec = new Mat(1, 1, CvType.CV_64FC3))
                    using (Mat corner_4x1 = detectedMarkerCorners[i].reshape(2, 4))
                    using (MatOfPoint2f imagePoints = new MatOfPoint2f(corner_4x1))
                    {
                        Calib3d.solvePnP(objectPoints, imagePoints, cameraIntrinsicMatrix, cameraDistortionCoeffs, rotationVec, translationVec);

                        double[] rvecArr = new double[3];
                        rotationVec.get(0, 0, rvecArr);
                        double[] tvecArr = new double[3];
                        translationVec.get(0, 0, tvecArr);

                        // Use OpenCVARUtils from OpenCVForUnity to convert OpenCV (Right-handed, Y-down) to Unity (Left-handed, Y-up)
                        PoseData poseData = OpenCVARUtils.ConvertRvecTvecToPoseData(rvecArr, tvecArr);
                        var arMatrix = OpenCVARUtils.ConvertPoseDataToMatrix(ref poseData, true);

                        // Apply the transformation to our target object relative to the main camera
                        if (mainCamera != null)
                        {
                            arMatrix = mainCamera.transform.localToWorldMatrix * arMatrix;
                        }
                        
                        OpenCVARUtils.SetTransformFromMatrix(targetObject.transform, ref arMatrix);
                    }
                }
            }
        }

        private void OnDestroy()
        {
            if (webCamTexture != null && webCamTexture.isPlaying)
            {
                webCamTexture.Stop();
            }

            if (rgbaMat != null) rgbaMat.Dispose();
            if (rgbMat != null) rgbMat.Dispose();
            if (cameraIntrinsicMatrix != null) cameraIntrinsicMatrix.Dispose();
            if (cameraDistortionCoeffs != null) cameraDistortionCoeffs.Dispose();
            if (detectedMarkerIds != null) detectedMarkerIds.Dispose();
            if (arucoDetector != null) arucoDetector.Dispose();
            
            foreach (var mat in detectedMarkerCorners) mat.Dispose();
            foreach (var mat in rejectedMarkerCandidates) mat.Dispose();
        }
    }
}
