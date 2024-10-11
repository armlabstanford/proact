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

using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Windows.WebCam;
using UnityEngine.UI;

namespace RosSharp.RosBridgeClient
{
    public class CapturingPublisher : UnityPublisher<MessageTypes.Sensor.CompressedImage>
    {
        public string FrameId = "Camera";
        [Range(0, 100)]
        public int qualityLevel = 50;

        private MessageTypes.Sensor.CompressedImage message;
        //private Texture2D texture2D;
        //private Rect rect;

        //private Matrix4x4 reflectToRH;
        //private Matrix4x4 rotateOpenCV2OpenGL;
        //private Matrix4x4 webcamToWorldROS;//remember not to send using conversion functions now!
        

        PhotoCapture photoCaptureObject = null;
        Texture2D targetTexture = null;
        //public RenderTexture renderTexture;
        public Transform cameraTransform;
        Matrix4x4 webcamToWorldMatrix = new Matrix4x4();
        Resolution cameraResolution;
        CameraParameters cameraParameters;
        bool PictureCompleted = true;
        public bool InitPhotoMode = false;

        private bool currentState;
        private bool previousState;

        private GameObject g;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();

            /*
            reflectToRH.SetRow(0, new Vector4(0, 0, 1, 0));
            reflectToRH.SetRow(1, new Vector4(-1, 0, 0, 0));
            reflectToRH.SetRow(2, new Vector4(0, 1, 0, 0));
            reflectToRH.SetRow(3, new Vector4(0, 0, 0, 1));
            */
            /*rotateOpenCV2OpenGL.SetRow(0, new Vector4(1, 0, 0, 0));
            rotateOpenCV2OpenGL.SetRow(1, new Vector4(0, -1, 0, 0));
            rotateOpenCV2OpenGL.SetRow(2, new Vector4(0, 0, -1, 0));
            rotateOpenCV2OpenGL.SetRow(3, new Vector4(0, 0, 0, 1));*/

            cameraResolution = PhotoCapture.SupportedResolutions.OrderByDescending((res) => res.width * res.height).First();
            targetTexture = new Texture2D(cameraResolution.width, cameraResolution.height);
            cameraParameters = new CameraParameters(WebCamMode.PhotoMode);
            cameraParameters.hologramOpacity = 0.0f;
            cameraParameters.cameraResolutionWidth = cameraResolution.width;
            cameraParameters.cameraResolutionHeight = cameraResolution.height;
            cameraParameters.pixelFormat = CapturePixelFormat.BGRA32;
            TakePicture();

            g = GameObject.Find("RosConnector");
            previousState = g.GetComponent<toggleResponder>().photoSwitch;

        }

        public void TakePicture()
        {
            // Create a PhotoCapture object
            PhotoCapture.CreateAsync(false, delegate (PhotoCapture captureObject) {
                photoCaptureObject = captureObject;
                // Activate the camera
                photoCaptureObject.StartPhotoModeAsync(cameraParameters, delegate (PhotoCapture.PhotoCaptureResult result) {
                    // Take a picture
                    InitPhotoMode = true;
                });
            });
        }

        public void TakePhoto()
        {
            if (PictureCompleted)
            {
                PictureCompleted = false;
                photoCaptureObject.TakePhotoAsync(OnCapturedPhotoToMemory);
            }
        }

        void Update()
        {
            currentState = g.GetComponent<toggleResponder>().photoSwitch;
            if (currentState != previousState)
            {
                TogglePhotoMode();
                previousState = currentState;
            }


            if (InitPhotoMode)
            {
                TakePhoto();
                UpdateMessage();
            }

            else return;

        }

        public bool TogglePhotoMode()
        {
            if (InitPhotoMode)
            {
                InitPhotoMode = false;
                photoCaptureObject.StopPhotoModeAsync(OnStoppedPhotoMode);
            }
            else
            {
                Start();
            }

            return InitPhotoMode;
        }

        void OnCapturedPhotoToMemory(PhotoCapture.PhotoCaptureResult result, PhotoCaptureFrame photoCaptureFrame)
        {
            photoCaptureFrame.TryGetCameraToWorldMatrix(out webcamToWorldMatrix);

            //webcamToWorldROS = reflectToRH * webcamToWorldMatrix;// * rotateOpenCV2OpenGL;

            cameraTransform.position = webcamToWorldMatrix.GetColumn(3);// - webcamToWorldMatrix.GetColumn(2); we don't want to display
            cameraTransform.rotation = Quaternion.LookRotation(-webcamToWorldMatrix.GetColumn(2), webcamToWorldMatrix.GetColumn(1));

            //cameraTransform.rotation = webcamToWorldROS.rotation;
            //cameraTransform.position = new Vector3(webcamToWorldROS[0, 3], webcamToWorldROS[1, 3], webcamToWorldROS[2, 3]);

            //showtext.text = "matrix now" + webcamToWorldMatrix.ToString();

            photoCaptureFrame.UploadImageDataToTexture(targetTexture);
            //Graphics.Blit(targetTexture, renderTexture);

            PictureCompleted = true;
        }

        void OnStoppedPhotoMode(PhotoCapture.PhotoCaptureResult result)
        {
            // Shutdown our photo capture resource
            photoCaptureObject.Dispose();
            photoCaptureObject = null;
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Sensor.CompressedImage();
            message.header.frame_id = FrameId;
            message.format = "jpeg";
        }

        private void UpdateMessage()
        {

            message.data = targetTexture.EncodeToJPG(qualityLevel);
            Publish(message);
        }


    }
}
