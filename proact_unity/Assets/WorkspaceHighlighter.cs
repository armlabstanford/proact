/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class WorkspaceHighlighter : UnitySubscriber<MessageTypes.Geometry.PoseStamped>
    {
        public Transform PublishedTransform;
        public Transform tableTransform;

        private Vector3 position;
        private Quaternion rotation;
        private bool isMessageReceived;

        protected override void Start()
        {
            base.Start();
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(MessageTypes.Geometry.PoseStamped message)
        {
            position = GetPosition(message).Ros2Unity();
            //rotation = GetRotation(message).Ros2Unity();
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            rotation.x = 0;
            rotation.y = 0;
            rotation.z = 0;
            rotation.w = 1;
            position.x = position.x; // + (float)0.121;
            position.z = position.z; // + (float)0.25;
            position.y = tableTransform.position.y; //+= (float)-0.5;
            PublishedTransform.position = position;
            PublishedTransform.rotation = rotation;
        }

        private Vector3 GetPosition(MessageTypes.Geometry.PoseStamped message)
        {
            print(message.pose.position.x);
            print(message.pose.position.y);
            print(message.pose.position.z);
            return new Vector3(
                (float)message.pose.position.x,
                (float)message.pose.position.y,
                (float)message.pose.position.z);
        }

        private Quaternion GetRotation(MessageTypes.Geometry.PoseStamped message)
        {
            return new Quaternion(
                (float)message.pose.orientation.x,
                (float)message.pose.orientation.y,
                (float)message.pose.orientation.z,
                (float)message.pose.orientation.w);
        }
    }
}
