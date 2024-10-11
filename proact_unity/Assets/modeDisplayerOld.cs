using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using UnityEditor;
using TMPro;

namespace RosSharp.RosBridgeClient
{
    public class ModeDisplayerOld : UnitySubscriber<std_msgs.Int8>
    {
        public GameObject v_x;
        public GameObject v_y;
        public GameObject v_z;
        public GameObject w_x;
        public GameObject w_y;
        public GameObject w_z;
        public GameObject shoulder1text;
        public GameObject shoulder2text;
        public GameObject elbow1text;
        public GameObject elbow2text;
        public GameObject wrist1text;
        public GameObject wrist2text;
        public GameObject wrist3text;
        public GameObject shoulder1;
        public GameObject shoulder2;
        public GameObject elbow1;
        public GameObject elbow2;
        public GameObject wrist1;
        public GameObject wrist2;
        public GameObject wrist3;
        public GameObject wristLink;
        public GameObject baseLink;
        public GameObject frameROS;
        public Material highlightMaterial;
        public Material normalMaterialArm;
        public Material normalMaterialHand;
        //public GameObject modeText;
        private int modeReceived;
        private bool isMessageReceived;

        protected override void Start()
        {
            base.Start();
            Debug.Log("Starting");
            vanish();
        }

        private void vanish()
        {
            frameROS.GetComponent<Renderer>().enabled = false;
            v_x.GetComponent<Renderer>().enabled = false;
            v_y.GetComponent<Renderer>().enabled = false;
            v_z.GetComponent<Renderer>().enabled = false;
            w_x.GetComponent<Renderer>().enabled = false;
            w_y.GetComponent<Renderer>().enabled = false;
            w_z.GetComponent<Renderer>().enabled = false;
            shoulder1text.GetComponent<Renderer>().enabled = false;
            elbow1text.GetComponent<Renderer>().enabled = false;
            wrist1text.GetComponent<Renderer>().enabled = false;
            shoulder2text.GetComponent<Renderer>().enabled = false;
            elbow2text.GetComponent<Renderer>().enabled = false;
            wrist2text.GetComponent<Renderer>().enabled = false;
            wrist3text.GetComponent<Renderer>().enabled = false;
            shoulder1.GetComponent<Renderer>().material = normalMaterialArm;
            shoulder2.GetComponent<Renderer>().material = normalMaterialArm;
            elbow1.GetComponent<Renderer>().material = normalMaterialArm;
            elbow2.GetComponent<Renderer>().material = normalMaterialArm;
            wrist1.GetComponent<Renderer>().material = normalMaterialHand;
            wrist2.GetComponent<Renderer>().material = normalMaterialHand;
            wrist3.GetComponent<Renderer>().material = normalMaterialHand;
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(std_msgs.Int8 message)
        {
            modeReceived = message.data;
            isMessageReceived = true;
        }

        private void onEEcontrol()
        {
            frameROS.transform.position = wristLink.transform.position;
            frameROS.transform.rotation = baseLink.transform.rotation;
            frameROS.GetComponent<Renderer>().enabled = true;
        }

        private void ProcessMessage()
        {
            vanish();
            //modeText.GetComponent<TMP_InputField>().text = modeReceived.ToString();
            // 1, 2, 3, 4,  for joint mode
            // 4, 5, 6, 7 for ee mode

            switch (modeReceived)
            {
                case 0:
                    shoulder1text.GetComponent<Renderer>().enabled = true;
                    shoulder1.GetComponent<Renderer>().material = highlightMaterial;
                    break;
                case 1:
                    shoulder2text.GetComponent<Renderer>().enabled = true;
                    shoulder2.GetComponent<Renderer>().material = highlightMaterial;
                    break;
                case 2:
                    elbow1text.GetComponent<Renderer>().enabled = true;
                    elbow1.GetComponent<Renderer>().material = highlightMaterial;
                    break;
                case 3:
                    elbow2text.GetComponent<Renderer>().enabled = true;
                    elbow2.GetComponent<Renderer>().material = highlightMaterial;
                    break;
                case 4:
                    wrist1text.GetComponent<Renderer>().enabled = true;
                    wrist1.GetComponent<Renderer>().material = highlightMaterial;
                    break;
                case 5:
                    wrist2text.GetComponent<Renderer>().enabled = true;
                    wrist2.GetComponent<Renderer>().material = highlightMaterial;
                    break;
                case 6:
                    wrist3text.GetComponent<Renderer>().enabled = true;
                    wrist3.GetComponent<Renderer>().material = highlightMaterial;
                    break;
                case 7:
                    onEEcontrol();
                    v_x.GetComponent<Renderer>().enabled = true;
                    break;
                case 8:
                    onEEcontrol();
                    v_y.GetComponent<Renderer>().enabled = true;
                    break;
                case 9:
                    onEEcontrol();
                    v_z.GetComponent<Renderer>().enabled = true;
                    break;
                case 10:
                    onEEcontrol();
                    w_x.GetComponent<Renderer>().enabled = true;
                    break;
                case 11:
                    onEEcontrol();
                    w_y.GetComponent<Renderer>().enabled = true;
                    break;
                case 12:
                    onEEcontrol();
                    w_z.GetComponent<Renderer>().enabled = true;
                    break;
            }
            isMessageReceived = false;
        }

    }
}
