using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using UnityEngine;

namespace PubSub
{
    public class Subscriber
    {
        private ConcurrentDictionary<string, Action<byte[]>> TopicCallbacks = new ConcurrentDictionary<string, Action<byte[]>>();

        private Thread _subThread;
        private readonly string _host;
        private readonly string _port;
        private bool _IsListening = false;

        private ConcurrentQueue<string> topicsToSubscribe = new ConcurrentQueue<string>();
        private ConcurrentQueue<string> topicsToUnsubscribe = new ConcurrentQueue<string>();

        public Subscriber(string host, string port, int maxBuffer = 10)
        {
            _host = host;
            _port = port;

            _IsListening = true;
            _subThread = new Thread(ListenerWork);
            _subThread.Start();
        }

        public void Dispose()
        {
            _IsListening = false;
            _subThread?.Join();
            _subThread = null;
        }

        ~Subscriber()
        {
            Dispose();
        }

        public void AddTopicCallback(string topic, Action<byte[]> callback)
        {
            topicsToSubscribe.Enqueue(topic);
            TopicCallbacks[topic] = callback;
        }

        public void RemoveTopic(string topic)
        {
            if (TopicCallbacks.ContainsKey(topic))
            {
                Debug.LogWarning("SubListener::RemoveTopic(): Removing Topic " + topic);
                TopicCallbacks.TryRemove(topic, out _);
                topicsToUnsubscribe.Enqueue(topic);
            }
        }

        private void ListenerWork()
        {
            AsyncIO.ForceDotNet.Force();
            using (var subSocket = new SubscriberSocket())
            {
                subSocket.Options.ReceiveHighWatermark = 10;
                subSocket.Connect($"tcp://{_host}:{_port}");

                while (_IsListening)
                {
                    while(!topicsToSubscribe.IsEmpty)
                        if (topicsToSubscribe.TryDequeue(out var newtopic))
                            subSocket.Subscribe(newtopic);

                    while (!topicsToUnsubscribe.IsEmpty)
                        if (topicsToUnsubscribe.TryDequeue(out var rmtopic)) 
                            subSocket.Unsubscribe(rmtopic);
                    try
                    {
                        if (subSocket.TryReceiveFrameString(out var topic))
                        {
                            if (TopicCallbacks.ContainsKey(topic) && subSocket.TryReceiveFrameBytes(out byte[] data) && data != null)
                            {
                                TopicCallbacks[topic](data);
                            }
                        }
                    }
                    catch (TerminatingException)
                    { }
                }
                subSocket.Dispose();
            }
            NetMQConfig.Cleanup();
        }
    }
}

