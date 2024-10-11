using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;

namespace RosSharp.RosBridgeClient
{
    public class GazeHitLocationPublisher : UnityPublisher<MessageTypes.Geometry.Vector3Stamped>
    {
        private Vector3 hitPos;
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
            hitPos = Microsoft.MixedReality.Toolkit.CoreServices.InputSystem.EyeGazeProvider.HitPosition.Unity2Ros();
            message.header.Update();
            message.vector.x = hitPos.x;
            message.vector.y = hitPos.y;
            message.vector.z = hitPos.z;
            Publish(message);
        }
    }
}
