using NetMQ;
using NetMQ.Sockets;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Vuforia;

public class NewBehaviourScript : MonoBehaviour
{
    [Header("AR Camera Settings")]
    [SerializeField] private GameObject arCamera;
    [SerializeField] private GameObject imageTarget;

    [Header("Network Settings")]
    [SerializeField] private string port = "55555";
    private PublisherSocket dataPubSocket;

    private ObserverBehaviour targetObserver;


    void Start()
    {
        InitializeSocket();

        targetObserver = imageTarget.GetComponent<ObserverBehaviour>();
        if (targetObserver == null)
        {
            Debug.LogError("Image Target does not have ObserverBehaviour attached.");
        }
    }

    private void InitializeSocket()
    {
        try
        {
            AsyncIO.ForceDotNet.Force();
            dataPubSocket = new PublisherSocket();
            dataPubSocket.Options.SendHighWatermark = 10;

            dataPubSocket.Bind($"tcp://*:{port}");
            Debug.Log("Successfully bound socket port " + port);
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to bind socket: {ex.Message}");
        }
    }

    void Update()
    {
        if (targetObserver.TargetStatus.Status == Status.TRACKED)
        {
            Matrix4x4 O2Marker = Matrix4x4.TRS(imageTarget.transform.position, imageTarget.transform.rotation, Vector3.one);
            Matrix4x4 O2Kinect = Matrix4x4.TRS(arCamera.transform.position, arCamera.transform.rotation, Vector3.one);

            Matrix4x4 Marker2Kinect = O2Marker.inverse * O2Kinect;

            //Matrix4x4 world2Marker = arCamera.transform.localToWorldMatrix * Marker2Kinect.inverse;

            byte[] matrixData = Matrix4x4ToByteArray(Marker2Kinect);

            PublishData("Calibration", matrixData);
        }
    }

    private void PublishData(string topic, byte[] data)
    {
        if (dataPubSocket != null)
        {
            try
            {
                dataPubSocket.SendMoreFrame(topic).SendFrame(data);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"Failed to publish data: {ex.Message}");
            }
        }
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

    void OnDestroy()
    {
        Debug.Log("Closing socket on port " + port);
        dataPubSocket.Dispose();
        NetMQConfig.Cleanup(false);
        dataPubSocket = null;
    }
}
