using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using UnityEditor;
using TMPro;

namespace RosSharp.RosBridgeClient
{
    public class ModeDisplayer : UnitySubscriber<std_msgs.Int8>
    {
        public GameObject v_x;
        public GameObject v_y;
        public GameObject v_z;
        public GameObject w_x;
        public GameObject w_y;
        public GameObject w_z;
        public GameObject shoulder1;
        public GameObject shoulder2;
        public GameObject elbow1;
        public GameObject elbow2;
        public GameObject wrist1;
        public GameObject wrist2;
        public GameObject wrist3;
        public GameObject x_axis;
        public GameObject y_axis;
        public GameObject z_axis;
        //public GameObject modeText;
        private int modeReceived;
        private bool isMessageReceived;
        private bool isMethodKnown;

        protected override void Start()
        {
            base.Start();
            Debug.Log("Starting");
            vanish();
            isMethodKnown = false;
        }

        private void vanish()
        {
            v_x.GetComponent<Renderer>().enabled = false;
            v_y.GetComponent<Renderer>().enabled = false;
            v_z.GetComponent<Renderer>().enabled = false;
            w_x.GetComponent<Renderer>().enabled = false;
            w_y.GetComponent<Renderer>().enabled = false;
            w_z.GetComponent<Renderer>().enabled = false;
            shoulder1.GetComponent<Renderer>().enabled = false;
            elbow1.GetComponent<Renderer>().enabled = false;
            wrist1.GetComponent<Renderer>().enabled = false;
            shoulder2.GetComponent<Renderer>().enabled = false;
            elbow2.GetComponent<Renderer>().enabled = false;
            wrist2.GetComponent<Renderer>().enabled = false;
            wrist3.GetComponent<Renderer>().enabled = false;
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

        private void hideMethodB()
        {
            x_axis.GetComponent<Renderer>().enabled = false;
            y_axis.GetComponent<Renderer>().enabled = false;
            z_axis.GetComponent<Renderer>().enabled = false;
            for (int i = 0; i < x_axis.GetComponent<Transform>().childCount; i++)
                x_axis.GetComponent<Transform>().GetChild(i).GetComponent<Renderer>().enabled = false;
            for (int i = 0; i < y_axis.GetComponent<Transform>().childCount; i++)
                y_axis.GetComponent<Transform>().GetChild(i).GetComponent<Renderer>().enabled = false;
            for (int i = 0; i < z_axis.GetComponent<Transform>().childCount; i++)
                z_axis.GetComponent<Transform>().GetChild(i).GetComponent<Renderer>().enabled = false;

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
                    shoulder1.GetComponent<Renderer>().enabled = true;
                    break;
                case 1:
                    shoulder2.GetComponent<Renderer>().enabled = true;
                    break;
                case 2:
                    elbow1.GetComponent<Renderer>().enabled = true;
                    break;
                case 3:
                    elbow2.GetComponent<Renderer>().enabled = true;
                    break;
                case 4:
                    wrist1.GetComponent<Renderer>().enabled = true;
                    break;
                case 5:
                    wrist2.GetComponent<Renderer>().enabled = true;
                    break;
                case 6:
                    wrist3.GetComponent<Renderer>().enabled = true;
                    break;                    
                case 7:
                    v_x.GetComponent<Renderer>().enabled = true;
                    break;
                case 8:
                    v_y.GetComponent<Renderer>().enabled = true;
                    break;
                case 9:
                    v_z.GetComponent<Renderer>().enabled = true;
                    break;
                case 10:
                    w_x.GetComponent<Renderer>().enabled = true;
                    break;
                case 11:
                    w_y.GetComponent<Renderer>().enabled = true;
                    break;
                case 12:
                    w_z.GetComponent<Renderer>().enabled = true;
                    break;
            }

            if (!isMethodKnown && modeReceived < 7)
            {
                hideMethodB();
                isMethodKnown = true;
            }

            isMessageReceived = false;
        }

    }
}
