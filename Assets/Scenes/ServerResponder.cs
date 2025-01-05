using System;
using System.Collections;
using System.Threading.Tasks;
using UnityEngine;
using Microsoft.Azure.Kinect.Sensor;
using NetMQ;
using NetMQ.Sockets;
using System.Linq;
using UnityEngine.Playables;
using PubSub;
using System.Reflection;
using TMPro;
using UnityEngine.UI;
using Vuforia;
using UnityEngine.UIElements;

public class ServerResponder : MonoBehaviour
{
    [Header("Network Settings")]
    [SerializeField] private string port = "12345";
    private ResponseSocket responseSocket;

    [Header("UI Display")]
    public RawImage ConnectionIndicator;

    [Header("ReadOnly and exposed for Debugging")]
    [SerializeField] protected Texture2D DepthImage;
    [SerializeField] protected Texture2D ColorInDepthImage;

    [Header("AR Camera Settings")]
    [SerializeField] private GameObject arCamera;
    [SerializeField] private GameObject imageTarget;
    [SerializeField] private GameObject model;
    private Matrix4x4 modelToCameraMatrix;

    [Header("Mode Selection")]
    [SerializeField] private Mode selectedMode;
    private enum Mode { Calibration, Play }

    private Device _Device;
    private Transformation kinectCalibration;

    private byte[] lookupData;
    private int lookupSplitSize = 500000;
    private byte[][] lookupParts = new byte[3][];

    private void Start()
    {
        if (selectedMode == Mode.Calibration)
        {
            // TODO: Manually disable Vuforia Delayed Initialization

            StartCoroutine(DetectAndTrackImageTarget());
        }
        else if (selectedMode == Mode.Play)
        {
            // TODO: Manually enable Vuforia Delayed Initialization

            // Disable Image Target
            if (imageTarget != null)
            {
                imageTarget.SetActive(false);
                Debug.Log("Image Target disabled");
            }

            InitializeSocket();
            StartCoroutine(CameraCaptureReplica());
        }
    }

    private void InitializeSocket()
    {
        try
        {
            AsyncIO.ForceDotNet.Force();
            responseSocket = new ResponseSocket();
            responseSocket.Bind($"tcp://*:{port}");
            Debug.Log("Successfully bound socket port " + port);
            ConnectionIndicator.color = Color.green;
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to bind socket: {ex.Message}");
        }
    }

    private IEnumerator DetectAndTrackImageTarget()
    {
        // Wait until the Image Target is detected
        ObserverBehaviour targetObserver = imageTarget.GetComponent<ObserverBehaviour>();
        if (targetObserver == null)
        {
            Debug.LogError("Image Target does not have ObserverBehaviour attached.");
            yield break;
        }

        Debug.Log("Waiting for Image Target to be tracked...");

        // Wait for Image Target to start tracking
        while (targetObserver.TargetStatus.Status != Status.TRACKED)
        {
            yield return null;
        }

        Debug.Log("Image Target detected. Starting to track...");

        // Continuously update the modelToCameraMatrix while the target is tracked
        while (targetObserver.TargetStatus.Status == Status.TRACKED)
        {
            // Retrieve and update the relative matrix of Model to Camera
            modelToCameraMatrix = arCamera.transform.worldToLocalMatrix * model.transform.localToWorldMatrix;
            Debug.Log(modelToCameraMatrix);

            yield return null; // Wait for the next frame
        }

        Debug.Log("Image Target is no longer tracked. Stopping updates.");
    }

    private IEnumerator CameraCaptureReplica()
    {
        if (Device.GetInstalledCount() == 0)
        {
            Debug.LogError("No Kinect Device Found");
            yield break;
        }

        try
        {
            _Device = Device.Open();
        }
        catch (AzureKinectOpenDeviceException ex)
        {
            Debug.LogError($"Failed to open Azure Kinect device: {ex.Message}");
            yield break;
        }

        var configuration = new DeviceConfiguration
        {
            ColorFormat = ImageFormat.ColorBGRA32,
            ColorResolution = ColorResolution.R1080p,
            DepthMode = DepthMode.NFOV_2x2Binned,
            SynchronizedImagesOnly = true,
            CameraFPS = FPS.FPS30
        };

        _Device.StartCameras(configuration);
        
        if (!SetupTextures(ref DepthImage, ref ColorInDepthImage))
        {
            Debug.LogError("CameraCaptureReplica(): Something went wrong while setting up camera textures");
            yield break;
        }

        kinectCalibration = _Device.GetCalibration(DepthMode.NFOV_2x2Binned, ColorResolution.R1080p).CreateTransformation();
        lookupData = GenerateXYTableData();

        while (true)
        {
            if (responseSocket.TryReceiveFrameString(out var request))
            {
                Debug.Log($"Received request: {request}");
                switch (request)
                {
                    case "Camera":
                        SendCameraData();
                        break;
                    case "Lookup1":
                        SendLookupData(1);
                        break;
                    case "Lookup2":
                        SendLookupData(2);
                        break;
                    case "Lookup3":
                        SendLookupData(3);
                        break;
                    case "Frame":
                        SendFrameData();
                        break;
                }
            }
            yield return null;
        }
    }

    private void SendCameraData()
    {
        var extrinsics = _Device.GetCalibration().DeviceExtrinsics[(int)CalibrationDeviceType.Depth + (int)CalibrationDeviceType.Color];
        Matrix4x4 extrinsics4x4 = new Matrix4x4();
        extrinsics4x4.SetRow(0, new Vector4(extrinsics.Rotation[0], extrinsics.Rotation[3], extrinsics.Rotation[6], extrinsics.Translation[0] / 1000.0f));
        extrinsics4x4.SetRow(1, new Vector4(extrinsics.Rotation[1], extrinsics.Rotation[4], extrinsics.Rotation[7], extrinsics.Translation[1] / 1000.0f));
        extrinsics4x4.SetRow(2, new Vector4(extrinsics.Rotation[2], extrinsics.Rotation[5], extrinsics.Rotation[8], extrinsics.Translation[2] / 1000.0f));
        extrinsics4x4.SetRow(3, new Vector4(0, 0, 0, 1));
        byte[] calibrationData = Matrix4x4ToByteArray(extrinsics4x4);

        byte[] cameraSizeData = null;
        try
        {
            using (var capture = _Device.GetCapture())
            {
                int[] captureArray = new int[6] {
                    capture.Color.WidthPixels, capture.Color.HeightPixels,
                    capture.Depth.WidthPixels, capture.Depth.HeightPixels,
                    capture.IR.WidthPixels, capture.IR.HeightPixels
                };

                cameraSizeData = new byte[captureArray.Length * sizeof(int)];
                Buffer.BlockCopy(captureArray, 0, cameraSizeData, 0, cameraSizeData.Length);
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning("Failed to get capture: " + ex.Message);
        }

        int cameraTotalSize = sizeof(int) * 2 + calibrationData.Length + cameraSizeData.Length;
        byte[] cameraData = new byte[cameraTotalSize];

        Buffer.BlockCopy(BitConverter.GetBytes(calibrationData.Length), 0, cameraData, 0, sizeof(int));
        Buffer.BlockCopy(BitConverter.GetBytes(cameraSizeData.Length), 0, cameraData, sizeof(int) * 1, sizeof(int));

        Buffer.BlockCopy(calibrationData, 0, cameraData, sizeof(int) * 2, calibrationData.Length);
        Buffer.BlockCopy(cameraSizeData, 0, cameraData, sizeof(int) * 2 + calibrationData.Length, cameraSizeData.Length);

        responseSocket.SendFrame(cameraData); // [calibrationData.Length][cameraSizeData.Length][calibrationData][cameraSizeData]
    }

    private void SendLookupData(int part)
    {
        int index = part - 1;
        int startIdx = index * lookupSplitSize;
        int length = Mathf.Min(lookupSplitSize, lookupData.Length - startIdx);
        lookupParts[index] = new byte[length];
        Array.Copy(lookupData, startIdx, lookupParts[index], 0, length);

        responseSocket.SendFrame(lookupParts[index]);   // [lookupData(part)]
    }

    private void SendFrameData()
    {
        using (var capture = _Device.GetCapture())
        {
            byte[] depthData = capture.Depth.Memory.ToArray();
            byte[] colorInDepthData = kinectCalibration.ColorImageToDepthCamera(capture).Memory.ToArray();

            DepthImage.LoadRawTextureData(depthData);
            DepthImage.Apply();
            ColorInDepthImage.LoadRawTextureData(colorInDepthData);
            ColorInDepthImage.Apply();

            int frameTotalSize = depthData.Length + colorInDepthData.Length + sizeof(int) * 2;
            byte[] frameData = new byte[frameTotalSize];

            Buffer.BlockCopy(BitConverter.GetBytes(depthData.Length), 0, frameData, 0, sizeof(int));
            Buffer.BlockCopy(BitConverter.GetBytes(colorInDepthData.Length), 0, frameData, sizeof(int), sizeof(int));

            Buffer.BlockCopy(depthData, 0, frameData, sizeof(int) * 2, depthData.Length);
            Buffer.BlockCopy(colorInDepthData, 0, frameData, sizeof(int) * 2 + depthData.Length, colorInDepthData.Length);

            long timestamp = DateTime.UtcNow.Ticks;
            byte[] timestampBytes = BitConverter.GetBytes(timestamp);

            byte[] message = new byte[timestampBytes.Length + frameData.Length];
            Buffer.BlockCopy(timestampBytes, 0, message, 0, timestampBytes.Length);
            Buffer.BlockCopy(frameData, 0, message, timestampBytes.Length, frameData.Length);

            responseSocket.SendFrame(message); // [timestamp (1 byte)][depthData.Length][colorInDepthData.Length][depthData][colorInDepthData]
        }
    }

    private bool SetupTextures(ref Texture2D Depth, ref Texture2D ColorInDepth)
    {
        try
        {
            using (var capture = _Device.GetCapture())
            {
                if (Depth == null)
                    Depth = new Texture2D(capture.Depth.WidthPixels, capture.Depth.HeightPixels, TextureFormat.R16, false);
                if (ColorInDepth == null)
                    ColorInDepth = new Texture2D(capture.IR.WidthPixels, capture.IR.HeightPixels, TextureFormat.BGRA32, false);
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"An error occurred " + ex.Message);
            return false;
        }
        return true;
    }

    private byte[] GenerateXYTableData()
    {
        var cal = _Device.GetCalibration();
        Texture2D xylookup = new Texture2D(DepthImage.width, DepthImage.height, TextureFormat.RGBAFloat, false);
        Vector2[] data = new Vector2[xylookup.width * xylookup.height];
        int idx = 0;

        System.Numerics.Vector2 p = new System.Numerics.Vector2();
        System.Numerics.Vector3? ray;

        for (int y = 0; y < xylookup.height; y++)
        {
            p.Y = y;
            for (int x = 0; x < xylookup.width; x++)
            {
                p.X = x;
                ray = cal.TransformTo3D(p, 1f, CalibrationDeviceType.Depth, CalibrationDeviceType.Depth);
                if (ray.HasValue)
                {
                    float xf = ray.Value.X;
                    float yf = ray.Value.Y;
                    data[idx].x = xf;
                    data[idx].y = yf;
                    xylookup.SetPixel(x, y, new Color((xf + 1) / 2, (yf + 1) / 2, 0, 0));
                }
                else
                {
                    xylookup.SetPixel(x, y, new Color(0, 0, 0, 0));
                    data[idx].x = 0;
                    data[idx].y = 0;
                }
            }
        }

        xylookup.Apply();
        byte[] xyTableData = xylookup.GetRawTextureData();
        return xyTableData;
    }

    private byte[] Matrix4x4ToByteArray(Matrix4x4 matrix)
    {
        float[] matrixFloats = new float[16];
        for (int i = 0; i < 16; i++)
        {
            matrixFloats[i] = matrix[i];
        }

        byte[] byteArray = new byte[matrixFloats.Length * sizeof(float)];
        Buffer.BlockCopy(matrixFloats, 0, byteArray, 0, byteArray.Length);
        return byteArray;
    }

    private void OnDestroy()
    {
        Debug.Log("Closing socket on port " + port);
        responseSocket.Dispose();
        NetMQConfig.Cleanup(false);
        responseSocket = null;

        StopAllCoroutines();
        Task.WaitAny(Task.Delay(1000));

        if (_Device != null)
        {
            _Device.StopCameras();
            _Device.Dispose();
        }
    }
}