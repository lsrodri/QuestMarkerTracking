using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

namespace TryAR.MarkerTracking
{
    /// <summary>
    /// ArUco marker detection and tracking component using Native C++ Plugin.
    /// Handles detection of ArUco markers in camera frames and provides pose estimation.
    /// </summary>
    public class ArUcoMarkerTracking : MonoBehaviour
    {
        [DllImport("aruco_unity_plugin", CallingConvention = CallingConvention.Cdecl)]
        private static extern int DetectMarkersAndEstimatePose(
            IntPtr imageData, int width, int height, int channels,
            int dictionaryId, float markerLength,
            float fx, float fy, float cx, float cy,
            [Out] MarkerData[] outMarkers, int maxOutMarkers);

        /// <summary>
        /// Available ArUco dictionaries for marker detection (from OpenCV objdetect)
        /// </summary>
        public enum ArUcoDictionary
        {
            DICT_4X4_50 = 0,
            DICT_4X4_100 = 1,
            DICT_4X4_250 = 2,
            DICT_4X4_1000 = 3,
            DICT_5X5_50 = 4,
            DICT_5X5_100 = 5,
            DICT_5X5_250 = 6,
            DICT_5X5_1000 = 7,
            DICT_6X6_50 = 8,
            DICT_6X6_100 = 9,
            DICT_6X6_250 = 10,
            DICT_6X6_1000 = 11,
            DICT_7X7_50 = 12,
            DICT_7X7_100 = 13,
            DICT_7X7_250 = 14,
            DICT_7X7_1000 = 15,
            DICT_ARUCO_ORIGINAL = 16,
        }

        [SerializeField] private ArUcoDictionary _dictionaryId = ArUcoDictionary.DICT_4X4_50;

        [Space(10)]

        [SerializeField] private float _markerLength = 0.1f;

        [Range(0, 1)]
        [SerializeField] private float _poseFilterCoefficient = 0.5f;

        [SerializeField] private int _divideNumber = 2;
        public int DivideNumber => _divideNumber;

        private float _fx, _fy, _cx, _cy;
        private int _imageWidth, _imageHeight;
        private bool _isReady = false;
        public bool IsReady => _isReady;

        private struct PoseData
        {
            public Vector3 Pos;
            public Quaternion Rot;
        }
        private Dictionary<int, PoseData> _prevPoseDataDictionary = new Dictionary<int, PoseData>();

        // Native plugin parameters
        private MarkerData[] _outMarkers;
        private int _maxMarkers = 10;
        private int _detectedMarkerCount = 0;

        private Texture2D m_cameraTexture;
        private RenderTexture _renderTexture;

        public void Initialize(int imageWidth, int imageHeight, float cx, float cy, float fx, float fy)
        {
            _imageWidth = imageWidth / _divideNumber;
            _imageHeight = imageHeight / _divideNumber;
            
            _fx = fx / _divideNumber;
            _fy = fy / _divideNumber;
            _cx = cx / _divideNumber;
            _cy = cy / _divideNumber;

            _outMarkers = new MarkerData[_maxMarkers];
            
            // Reusable textures for downscaling and reading
            m_cameraTexture = new Texture2D(_imageWidth, _imageHeight, TextureFormat.RGBA32, false);
            _renderTexture = new RenderTexture(_imageWidth, _imageHeight, 0, RenderTextureFormat.ARGB32);
            
            _isReady = true;
        }

        public void DetectMarker(Texture webCamTexture, Texture2D resultTexture = null)
        {
            if (!_isReady || webCamTexture == null) return;

            // 1. Efficiently scale down and extract pixels from generic Texture
            RenderTexture prev = RenderTexture.active;
            RenderTexture.active = _renderTexture;
            Graphics.Blit(webCamTexture, _renderTexture);
            m_cameraTexture.ReadPixels(new Rect(0, 0, _imageWidth, _imageHeight), 0, 0);
            m_cameraTexture.Apply();
            RenderTexture.active = prev;

            // 2. Pass bytes to C++ via zero-copy native pointer
            NativeArray<byte> textureData = m_cameraTexture.GetRawTextureData<byte>();
            
            unsafe
            {
                IntPtr ptr = (IntPtr)NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(textureData);
                
                try
                {
                    _detectedMarkerCount = DetectMarkersAndEstimatePose(
                        ptr, _imageWidth, _imageHeight, 4,
                        (int)_dictionaryId, _markerLength,
                        _fx, _fy, _cx, _cy,
                        _outMarkers, _maxMarkers);
                }
                catch (Exception e)
                {
                    Debug.LogError($"Native plugin error: {e.Message}");
                    _detectedMarkerCount = 0;
                }
            }

            // Optional visualization (copy texture over)
            if (resultTexture != null)
            {
                Graphics.CopyTexture(m_cameraTexture, resultTexture);
            }
        }

        public void EstimatePoseCanonicalMarker(Dictionary<int, GameObject> arObjects, Transform camTransform)
        {
            if (!_isReady || _detectedMarkerCount == 0) return;

            for (int i = 0; i < _detectedMarkerCount; i++)
            {
                int currentMarkerId = _outMarkers[i].id;

                if (!arObjects.TryGetValue(currentMarkerId, out GameObject targetObject) || targetObject == null)
                    continue;

                Vector3 currentPos = new Vector3(_outMarkers[i].posX, _outMarkers[i].posY, _outMarkers[i].posZ);
                Quaternion currentRot = new Quaternion(_outMarkers[i].rotX, _outMarkers[i].rotY, _outMarkers[i].rotZ, _outMarkers[i].rotW);

                PoseData poseData = new PoseData { Pos = currentPos, Rot = currentRot };

                if (!_prevPoseDataDictionary.TryGetValue(currentMarkerId, out PoseData prevPose))
                {
                    prevPose = new PoseData();
                    _prevPoseDataDictionary[currentMarkerId] = prevPose;
                }

                if (prevPose.Pos != Vector3.zero)
                {
                    float t = _poseFilterCoefficient;
                    poseData.Pos = Vector3.Lerp(poseData.Pos, prevPose.Pos, t);
                    poseData.Rot = Quaternion.Slerp(poseData.Rot, prevPose.Rot, t);
                }

                _prevPoseDataDictionary[currentMarkerId] = poseData;

                Matrix4x4 arMatrix = Matrix4x4.TRS(poseData.Pos, poseData.Rot, Vector3.one);
                arMatrix = camTransform.localToWorldMatrix * arMatrix;

                targetObject.transform.position = arMatrix.GetColumn(3);
                targetObject.transform.rotation = arMatrix.rotation;
            }
        }

        public void Dispose()
        {
            ReleaseResources();
        }

        void OnDestroy()
        {
            ReleaseResources();
        }

        private void ReleaseResources()
        {
            if (_renderTexture != null)
            {
                _renderTexture.Release();
                Destroy(_renderTexture);
            }
            if (m_cameraTexture != null)
            {
                Destroy(m_cameraTexture);
            }
        }
    }
}