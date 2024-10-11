using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient
{
    public class GazeTargetPublisher : UnityPublisher<std_msgs.String>
    {
        private string gazeTarget;
        private std_msgs.String message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void InitializeMessage()
        {
            message = new std_msgs.String();
        }

        private void Update()
        {
            gazeTarget = Microsoft.MixedReality.Toolkit.CoreServices.InputSystem.EyeGazeProvider.GazeTarget.name;
            message.data = gazeTarget;
            Publish(message);
        }
    }
}
