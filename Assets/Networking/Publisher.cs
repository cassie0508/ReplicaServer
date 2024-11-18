using UnityEngine;
using NetMQ;
using NetMQ.Sockets;

namespace PubSub
{
    public class Publisher : MonoBehaviour
    {
        public static Publisher Instance { get; private set; }

        [SerializeField] private string port;

        private PublisherSocket dataPubSocket;

        private void Awake()
        {
            Instance = this;

            AsyncIO.ForceDotNet.Force();
            dataPubSocket = new PublisherSocket();
            dataPubSocket.Options.TcpKeepalive = false;
            dataPubSocket.Options.SendHighWatermark = 10;

            Debug.Log("Publisher::Awake(): Binding socket to port " + port);
            dataPubSocket.Bind($"tcp://*:{port}");
        }

        private void OnDestroy()
        {
            Debug.Log("Publisher::OnDestroy(): Closing socket on port " + port);
            dataPubSocket.Dispose();
            NetMQConfig.Cleanup(false);
            dataPubSocket = null;
        }

        public bool PublishData(string topic, byte[] data)
        {
            if (dataPubSocket != null)
            {
                try
                {
                    dataPubSocket?.SendMoreFrame(topic).SendFrame(data);

                    return true;
                }
                catch { }
            }

            return false;
        }
    }
}