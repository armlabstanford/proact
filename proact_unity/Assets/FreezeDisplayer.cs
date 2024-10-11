using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using UnityEditor;

namespace RosSharp.RosBridgeClient
{
    public class FreezeDisplayer : UnitySubscriber<std_msgs.Bool>
    {
        public GameObject pickHighlighterObject;
        public GameObject placeHighligherObject;
        private bool frozen = false;
        private bool isMessageReceived;
        public Material frozenPickMaterial;
        public Material frozenPlaceMaterial;
        public Material unfrozenPickMaterial;
        public Material unfrozenPlaceMaterial;

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

        protected override void ReceiveMessage(std_msgs.Bool message)
        {
            frozen = message.data;
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            if (frozen)
            {
                pickHighlighterObject.GetComponent<Renderer>().material = frozenPickMaterial;
                placeHighligherObject.GetComponent<Renderer>().material = frozenPlaceMaterial;
            }
            else
            {
                pickHighlighterObject.GetComponent<Renderer>().material = unfrozenPickMaterial;
                placeHighligherObject.GetComponent<Renderer>().material = unfrozenPlaceMaterial;
            }
            isMessageReceived = false; //so things happen only once each time
        }

    }
}
