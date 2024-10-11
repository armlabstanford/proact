    using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using UnityEditor;

namespace RosSharp.RosBridgeClient
{
    public class BlockHighlighter : UnitySubscriber<std_msgs.String>
    {
        public GameObject highlighterObject;
        private string stringReceived;
        private bool isMessageReceived;

        protected override void Start()
        {
            base.Start();
            Debug.Log("Starting");
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(std_msgs.String message)
        {
            stringReceived = message.data;
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            highlighterObject.transform.position = GameObject.Find(stringReceived).transform.position;
        }

    }
}
