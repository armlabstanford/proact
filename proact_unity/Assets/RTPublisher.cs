/*Modified by Shivani Guptasarma
 * shivanig@stanford.edu
 * 
© CentraleSupelec, 2017
Author: Dr. Jeremy Fix (jeremy.fix@centralesupelec.fr)

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

// Adjustments to new Publication Timing and Execution Framework 
// © Siemens AG, 2018, Dr. Martin Bischoff (martin.bischoff@siemens.com)

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class RTPublisher : UnityPublisher<MessageTypes.Sensor.CompressedImage>
    {
        public RenderTexture renderTexture;
        public RenderTexture debuggerTexture;
        //public GameObject logger;
        public string FrameId = "Camera";
        public int resolutionWidth = 640;
        public int resolutionHeight = 480;
        [Range(0, 100)]
        public int qualityLevel = 50;

        private MessageTypes.Sensor.CompressedImage message;
        private Texture2D texture2D;
        private Rect rect;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
            texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.BGRA32, false);
            rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
            debuggerTexture = new RenderTexture(resolutionWidth, resolutionHeight, 0);
        }

        void Update()
        {
            //RenderTexture.active = renderTexture;
            //texture2D.ReadPixels(rect, 0, 0);
            //texture2D.Apply();
            //message.data = texture2D.EncodeToJPG(qualityLevel);
            //message.data = (logger.GetComponent<Logger>().targetTexture).EncodeToJPG(qualityLevel);
            //Publish(message);
            UpdateMessage();
        }

        
        private void InitializeMessage()
        {
            message = new MessageTypes.Sensor.CompressedImage();
            message.header.frame_id = FrameId;
            message.format = "jpeg";
        }
        
        private void UpdateMessage()
        {
            RenderTexture.active = renderTexture;
            texture2D.ReadPixels(rect, 0, 0);
            //texture2D.Apply();

            //for debugging
            RenderTexture.active = debuggerTexture;
            Graphics.Blit(texture2D, debuggerTexture);

            message.data = texture2D.EncodeToJPG(qualityLevel);
            Publish(message);
        }
        

    }
}
