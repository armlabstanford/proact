/* Shivani Guptasarma, shivanig@stanford.edu, 
 * Using code from Carlos Lopez 
 */

using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Windows.WebCam;
using UnityEngine.UI;
using TMPro;


namespace RosSharp.RosBridgeClient
{
    public class LoggerCopy : MonoBehaviour
    {
        PhotoCapture photoCaptureObject = null;
        public Texture2D targetTexture = null;

        public Transform transformObject;

        Resolution cameraResolution;
        public CameraParameters cameraParameters;
        bool PictureCompleted = true;
        public bool InitPhotoMode = false;

        private bool currentState;
        private bool previousState;

        private GameObject g;


        // Start is called before the first frame update
        void Start()
        {
            cameraResolution = PhotoCapture.SupportedResolutions.OrderByDescending((res) => res.width * res.height).First();
            targetTexture = new Texture2D(cameraResolution.width, cameraResolution.height);
            cameraParameters = new CameraParameters(WebCamMode.PhotoMode);
            cameraParameters.hologramOpacity = 0.0f;
            cameraParameters.cameraResolutionWidth = cameraResolution.width;
            cameraParameters.cameraResolutionHeight = cameraResolution.height;
            cameraParameters.pixelFormat = CapturePixelFormat.BGRA32;
            TakePicture();

            g = GameObject.Find("RosConnector1");
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
            }

            else return;

        }

        public void TogglePhotoMode()
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
        }

        void OnCapturedPhotoToMemory(PhotoCapture.PhotoCaptureResult result, PhotoCaptureFrame photoCaptureFrame)
        {

            Matrix4x4 webcamToWorldMatrix = new Matrix4x4();
            bool success = photoCaptureFrame.TryGetCameraToWorldMatrix(out webcamToWorldMatrix);

            transformObject.position = webcamToWorldMatrix.GetColumn(3);
            transformObject.rotation = Quaternion.LookRotation(-webcamToWorldMatrix.GetColumn(2), webcamToWorldMatrix.GetColumn(1));

            photoCaptureFrame.UploadImageDataToTexture(targetTexture);

            PictureCompleted = true;
        }

        void OnStoppedPhotoMode(PhotoCapture.PhotoCaptureResult result)
        {
            // Shutdown our photo capture resource
            photoCaptureObject.Dispose();
            photoCaptureObject = null;
        }
    }

}

