using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using UnityEditor;

namespace RosSharp.RosBridgeClient
{
    public class PlanSuccessDisplayer : UnitySubscriber<std_msgs.Bool>
    {
        public GameObject highlightedObject;
        private bool planned = false;
        private bool isMessageReceived;
        public Material yesMaterial;
        public Material noMaterial;
        public Material genMaterial;
        int flashCount = 0;
        bool flashingNow = false;

        protected override void Start()
        {
            base.Start();
            Debug.Log("Starting");
        }

        private void Update()
        {
            if (flashCount == 4) //reset after n-1 frames
            {
                flashCount = 0;
                flashingNow = false; //stop counting until next message received
                highlightedObject.GetComponent<Renderer>().material = genMaterial;
            }
            if (isMessageReceived)
                ProcessMessage();
            if (flashingNow)
                flashCount++;
        }

        protected override void ReceiveMessage(std_msgs.Bool message)
        {
            planned = message.data;
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            if (planned)
            {
                highlightedObject.GetComponent<Renderer>().material = yesMaterial;
            }
            else
            {
                highlightedObject.GetComponent<Renderer>().material = noMaterial;
            }
            isMessageReceived = false; //so things happen only once each time
            flashingNow = true;
        }

    }
}
