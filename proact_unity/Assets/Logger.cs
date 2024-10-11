//Original script by Carlos Lopez, Microsoft Support Center, Dec 2021
//Modified by Shivani Guptasarma shivanig@stanford.edu

using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Windows.WebCam;
using UnityEngine.UI;
//using TMPro;

public class Logger : MonoBehaviour
{
    //[SerializeField] public TextMeshProUGUI mText;
    //public Text showtext;
    PhotoCapture photoCaptureObject = null;
    Texture2D targetTexture = null;
    public RenderTexture renderTexture;
    public Transform cameraTransform;
    Matrix4x4 webcamToWorldMatrix = new Matrix4x4();
    Resolution cameraResolution;
    CameraParameters cameraParameters;
    bool PictureCompleted = true;
    bool InitPhotoMode = false;

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
        if (!InitPhotoMode) return;

        TakePhoto();
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
        photoCaptureFrame.TryGetCameraToWorldMatrix(out webcamToWorldMatrix);

        cameraTransform.rotation = webcamToWorldMatrix.rotation;
        cameraTransform.position = new Vector3(webcamToWorldMatrix[0, 3], webcamToWorldMatrix[1, 3], webcamToWorldMatrix[2, 3]);

        //showtext.text = "matrix now" + webcamToWorldMatrix.ToString();

        photoCaptureFrame.UploadImageDataToTexture(targetTexture);
        Graphics.Blit(targetTexture, renderTexture);

        PictureCompleted = true;
    }

    void OnStoppedPhotoMode(PhotoCapture.PhotoCaptureResult result)
    {
        // Shutdown our photo capture resource
        photoCaptureObject.Dispose();
        photoCaptureObject = null;
    }
}
