using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;

namespace RosSharp.RosBridgeClient
{
    public class GazeDirectionPublisher : UnityPublisher<MessageTypes.Geometry.Vector3Stamped>
    {
        private Vector3 eyeDir;
        private MessageTypes.Geometry.Vector3Stamped message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Geometry.Vector3Stamped();
        }

        private void Update()
        {
            eyeDir = Microsoft.MixedReality.Toolkit.CoreServices.InputSystem.EyeGazeProvider.GazeDirection.Unity2Ros();
            message.header.Update();
            message.vector.x = eyeDir.x;
            message.vector.y = eyeDir.y;
            message.vector.z = eyeDir.z;
            Publish(message);
        }
    }
}
