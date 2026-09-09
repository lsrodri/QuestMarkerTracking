using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

namespace TryAR.MarkerTracking
{
    [StructLayout(LayoutKind.Sequential)]
    public struct MarkerData
    {
        public int id;
        public float posX;
        public float posY;
        public float posZ;
        public float rotX;
        public float rotY;
        public float rotZ;
        public float rotW;
    }

    /// <summary>
    /// Phase 3: Final Editor Baseline using the custom C++ Native Plugin (aruco_unity_plugin).
    /// </summary>
    public class EditorWebcamArucoTester : MonoBehaviour
    {
        [DllImport("aruco_unity_plugin", CallingConvention = CallingConvention.Cdecl)]
        private static extern int DetectMarkersAndEstimatePose(
            IntPtr imageData, int width, int height, int channels,
            int dictionaryId, float markerLength,
            float fx, float fy, float cx, float cy,
            [Out] MarkerData[] outMarkers, int maxOutMarkers);

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
        private Color32[] colors;
        
        // Output array for P/Invoke
        private MarkerData[] outMarkers;
        private int maxMarkers = 10;
        
        // Pinned array for colors
        private GCHandle colorsHandle;

        void Start()
        {
            if (mainCamera == null)
                mainCamera = Camera.main;

            outMarkers = new MarkerData[maxMarkers];

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
                if (colors == null || colors.Length != webCamTexture.width * webCamTexture.height)
                {
                    if (colorsHandle.IsAllocated)
                        colorsHandle.Free();
                        
                    colors = new Color32[webCamTexture.width * webCamTexture.height];
                    colorsHandle = GCHandle.Alloc(colors, GCHandleType.Pinned);
                }

                ProcessFrame();
            }
        }

        private void ProcessFrame()
        {
            // 1. Get raw bytes directly from WebCamTexture
            // We use GetPixels32 instead of GetRawTextureData to ensure consistent format
            webCamTexture.GetPixels32(colors);

            int width = webCamTexture.width;
            int height = webCamTexture.height;
            int channels = 4; // Color32 is RGBA (4 bytes per pixel)

            // 2. Setup Camera Intrinsics
            float fx = width, fy = width, cx = width / 2.0f, cy = height / 2.0f;
            if (useUnityCameraIntrinsics && mainCamera != null)
            {
                float fovY = mainCamera.fieldOfView;
                float fovX = 2.0f * Mathf.Atan(Mathf.Tan(fovY * Mathf.Deg2Rad * 0.5f) * mainCamera.aspect) * Mathf.Rad2Deg;
                fx = (width / 2.0f) / Mathf.Tan(fovX * Mathf.Deg2Rad * 0.5f);
                fy = (height / 2.0f) / Mathf.Tan(fovY * Mathf.Deg2Rad * 0.5f);
                cx = width / 2.0f;
                cy = height / 2.0f;
            }

            // 3. Invoke Native Plugin
            IntPtr ptr = colorsHandle.AddrOfPinnedObject();
            
            try
            {
                int detectedCount = DetectMarkersAndEstimatePose(
                    ptr, width, height, channels,
                    (int)dictionaryId, markerLength,
                    fx, fy, cx, cy,
                    outMarkers, maxMarkers);

                // 4. Apply results
                if (detectedCount > 0 && targetObject != null)
                {
                    for (int i = 0; i < detectedCount; i++)
                    {
                        if (outMarkers[i].id == targetMarkerId)
                        {
                            Vector3 pos = new Vector3(outMarkers[i].posX, outMarkers[i].posY, outMarkers[i].posZ);
                            Quaternion rot = new Quaternion(outMarkers[i].rotX, outMarkers[i].rotY, outMarkers[i].rotZ, outMarkers[i].rotW);

                            Matrix4x4 arMatrix = Matrix4x4.TRS(pos, rot, Vector3.one);

                            if (mainCamera != null)
                            {
                                arMatrix = mainCamera.transform.localToWorldMatrix * arMatrix;
                            }

                            targetObject.transform.position = arMatrix.GetColumn(3);
                            targetObject.transform.rotation = arMatrix.rotation;
                            targetObject.transform.localScale = arMatrix.lossyScale;
                            break;
                        }
                    }
                }
            }
            catch (DllNotFoundException e)
            {
                Debug.LogError($"Native plugin not found. Please build aruco_unity_plugin.dll and place it in Assets/Plugins. Error: {e.Message}");
            }
            catch (EntryPointNotFoundException e)
            {
                Debug.LogError($"Function not found in the native plugin. Error: {e.Message}");
            }
        }

        private void OnDestroy()
        {
            if (webCamTexture != null && webCamTexture.isPlaying)
            {
                webCamTexture.Stop();
            }

            if (colorsHandle.IsAllocated)
            {
                colorsHandle.Free();
            }
        }
    }
}
