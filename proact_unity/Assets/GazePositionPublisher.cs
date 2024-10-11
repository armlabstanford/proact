using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;

namespace RosSharp.RosBridgeClient
{
    public class GazePositionPublisher : UnityPublisher<MessageTypes.Geometry.Vector3Stamped>
    {
        private Vector3 eyePos;
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
            eyePos = Microsoft.MixedReality.Toolkit.CoreServices.InputSystem.EyeGazeProvider.GazeOrigin.Unity2Ros();
            message.header.Update();
            message.vector.x = eyePos.x;
            message.vector.y = eyePos.y;
            message.vector.z = eyePos.z;
            Publish(message);
        }
    }
}