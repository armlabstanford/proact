using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using UnityEditor;

namespace RosSharp.RosBridgeClient
{
    public class GraspStatusDisplayer : UnitySubscriber<std_msgs.Bool>
    {
        public GameObject highlighterObject;
        public AudioSource contactMade;
        public AudioSource contactLost;
        private bool grasped;
        private bool isMessageReceived;

        protected override void Start()
        {
            base.Start();
            Debug.Log("Starting");
            grasped = false;
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(std_msgs.Bool message)
        {
            grasped = message.data;
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            if (grasped)
            {
                highlighterObject.GetComponent<Renderer>().enabled = false;
                contactMade.Play();
            }
            else
            {
                highlighterObject.GetComponent<Renderer>().enabled = true;
                contactLost.Play();
            }
            isMessageReceived = false; //so things happen only once each time
        }

    }
}
