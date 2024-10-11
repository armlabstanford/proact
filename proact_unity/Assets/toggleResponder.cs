/*
Shivani Guptasarma
shivanig@stanford.edu

Toggle photo capture on/off in response to a ROS service call (photo capture is used to find the transform bw real world and Hololens world coordinate system)
*/

using System.Collections.Generic;
using System;
using UnityEngine;
using RosSharp.RosBridgeClient;
using std_srvs = RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient
{
    public class toggleResponder : MonoBehaviour
    {
        public bool photoSwitch = true;

        void Start()
        {
            string service_id = GetComponent<RosConnector>().RosSocket.AdvertiseService<std_srvs.TriggerRequest, std_srvs.TriggerResponse>("/service_response_test", ServiceResponseHandler);
            photoSwitch = true;
        }

        private bool ServiceResponseHandler(std_srvs.TriggerRequest arguments, out std_srvs.TriggerResponse result)
        {
            photoSwitch = !photoSwitch;
            result = new std_srvs.TriggerResponse(true, "service response message");
            return true;
        }
    }
}