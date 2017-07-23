//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

using System;
using System.Collections;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using Windows.ApplicationModel;
using Windows.Devices.Enumeration;
using Windows.Graphics.Display;
using Windows.Graphics.Imaging;
using Windows.Media;
using Windows.Media.Capture;
using Windows.Media.Core;
using Windows.Media.MediaProperties;
using Windows.Media.Playback;
using Windows.Storage;
using Windows.System.Display;
using Windows.UI;
using Windows.UI.Core;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media.Imaging;
using Windows.UI.Xaml.Navigation;

using CameraGetPreviewFrame.Models;

namespace CameraGetPreviewFrame
{
    [ComImport]
    [Guid("5b0d3235-4dba-4d44-865e-8f1d0e4fd04d")]
    [InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    unsafe interface IMemoryBufferByteAccess
    {
        void GetBuffer(out byte* buffer, out uint capacity);
    }

    public sealed partial class MainPage : Page
    {
        // Receive notifications about rotation of the UI and apply any necessary rotation to the preview stream
        private readonly DisplayInformation _displayInformation = DisplayInformation.GetForCurrentView();
        private DisplayOrientations _displayOrientation = DisplayOrientations.Portrait;

        // Rotation metadata to apply to the preview stream (MF_MT_VIDEO_ROTATION)
        // Reference: http://msdn.microsoft.com/en-us/library/windows/apps/xaml/hh868174.aspx
        private static readonly Guid RotationKey = new Guid("C380465D-2271-428C-9B83-ECEA3B4A85C1");

        // Folder in which the captures will be stored (initialized in InitializeCameraAsync)
        private StorageFolder _captureFolder = null;

        // Prevent the screen from sleeping while the camera is running
        private readonly DisplayRequest _displayRequest = new DisplayRequest();

        // For listening to media property changes
        private readonly SystemMediaTransportControls _systemMediaControls = SystemMediaTransportControls.GetForCurrentView();

        // MediaCapture and its state variables
        private MediaCapture _mediaCapture;
        private bool _isInitialized = false;
        private bool _isPreviewing = false;

        // Information about the camera device
        private bool _mirroringPreview = false;
        private bool _externalCamera = false;

        // Fujimaki Add
        private string[] _cameraDeviceNames =
        {
            "Microsoft LifeCam Rear",
            "Microsoft LifeCam Front",
            "USB 2.0 Camera",
            "USB 2.0 HD-720P web cam"
        };
#if DEBUG
        private int _cameraID = 1;
#else
        private int _cameraID = 2;
#endif


        // Fujimaki Add タイマー
        private DispatcherTimer _timer;
        // Fujimaki Add タイマーの時間間隔[ms]
        private int _timerInterval = 100;

        // Fujimaki Add 色合いのヒストグラムのピークを認識する閾値
        private double _hueThreshold = 300.0;

        // Fujimaki Add ヒストグラムの平滑化フィルタの長さ
        private int _smoothFilterLength = 11;

        // Fujimaki Add 音声ファイル名
        private ArrayList _soundFileNames = null;

        // Fujimaki Add 楽器名
        private string[] _gakkiNames = new string[] {
            "piano",
            "guitar",
            "echo2s",
            "synthe"
        };

        // 1つの楽器に割り当てる音声ファイルの数
        private int _numSoundPerGakki = 8;

        private MediaPlayer[,] _soundPlayer = null;
        private bool[,] _playEnable = null;

        #region Constructor, lifecycle and navigation

        public MainPage()
        {
            this.InitializeComponent();

            // Cache the UI to have the checkboxes retain their state, as the enabled/disabled state of the
            // GetPreviewFrameButton is reset in code when suspending/navigating (see Start/StopPreviewAsync)
            NavigationCacheMode = NavigationCacheMode.Required;

            // Useful to know when to initialize/clean up the camera
            Application.Current.Suspending += Application_Suspending;
            Application.Current.Resuming += Application_Resuming;

            // Fujimaki Add 音声ファイル名を設定する。
            _soundFileNames = new ArrayList();
            for (int iGakki = 0; iGakki < _gakkiNames.Length; iGakki++)
            {
                _soundFileNames.Add(new ArrayList());
                for (int i = 1; i <= _numSoundPerGakki; i++)
                {
                    ((ArrayList)_soundFileNames[iGakki]).Add(_gakkiNames[iGakki] + i.ToString("D2") + ".mp3");
                }
            }

            // Fujimaki Add
            _soundPlayer = new MediaPlayer[_soundFileNames.Count, _numSoundPerGakki];    
            _playEnable = new bool[_soundFileNames.Count, _numSoundPerGakki];

            // Fujimaki Add
            for (int iGakki = 0; iGakki < _gakkiNames.Length; iGakki++)
            {
                for (int i = 0; i < _numSoundPerGakki; i++)
                {
                    _soundPlayer[iGakki, i] = new MediaPlayer();
                    _soundPlayer[iGakki, i].AutoPlay = false;
                    _soundPlayer[iGakki, i].Source = MediaSource.CreateFromUri(
                        new Uri(this.BaseUri, "Assets/" + _gakkiNames[iGakki] + "/" + ((ArrayList)_soundFileNames[iGakki])[i])
                        );
                }
            }
        }

        private async void Application_Suspending(object sender, SuspendingEventArgs e)
        {
            // Handle global application events only if this page is active
            if (Frame.CurrentSourcePageType == typeof(MainPage))
            {
                var deferral = e.SuspendingOperation.GetDeferral();

                await CleanupCameraAsync();

                _displayInformation.OrientationChanged -= DisplayInformation_OrientationChanged;

                deferral.Complete();
            }
        }

        private async void Application_Resuming(object sender, object o)
        {
            // Handle global application events only if this page is active
            if (Frame.CurrentSourcePageType == typeof(MainPage))
            {
                // Populate orientation variables with the current state and register for future changes
                _displayOrientation = _displayInformation.CurrentOrientation;
                _displayInformation.OrientationChanged += DisplayInformation_OrientationChanged;

                await InitializeCameraAsync();
            }
        }

        protected override async void OnNavigatedTo(NavigationEventArgs e)
        {
            // Populate orientation variables with the current state and register for future changes
            _displayOrientation = _displayInformation.CurrentOrientation;
            _displayInformation.OrientationChanged += DisplayInformation_OrientationChanged;

            await InitializeCameraAsync();

            //Task.Delay(5000).Wait();    // Fujimaki Add

            // Fujimaki Add
            while (_soundPlayer == null)
            {
                // 何もせずにループする。
                Task.Delay(100).Wait();
            }
            for (int iGakki = 0; iGakki < _gakkiNames.Length; iGakki++)
            {
                for (int i = 0; i < _numSoundPerGakki; i++)
                {
                    while (_soundPlayer[iGakki, i] == null || _soundPlayer[iGakki, i].PlaybackSession.PlaybackState == MediaPlaybackState.Opening)
                    {
                        // 何もせずにループする。
                        Task.Delay(100).Wait();
                    }
                }
            }

            _timer = new DispatcherTimer(); // Fujimaki Add
            _timer.Interval = TimeSpan.FromMilliseconds(_timerInterval);    // Fujimaki Add
            _timer.Tick += _timer_Tick; // Fujimaki Add
            _timer.Start(); // Fujimaki Add
        }

        protected override async void OnNavigatingFrom(NavigatingCancelEventArgs e)
        {
            // Handling of this event is included for completenes, as it will only fire when navigating between pages and this sample only includes one page

            _timer.Stop();
            await CleanupCameraAsync();

            _displayInformation.OrientationChanged -= DisplayInformation_OrientationChanged;
        }

        #endregion Constructor, lifecycle and navigation


        #region Event handlers

        /// <summary>
        /// In the event of the app being minimized this method handles media property change events. If the app receives a mute
        /// notification, it is no longer in the foregroud.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="args"></param>
        private async void SystemMediaControls_PropertyChanged(SystemMediaTransportControls sender, SystemMediaTransportControlsPropertyChangedEventArgs args)
        {
            await Dispatcher.RunAsync(CoreDispatcherPriority.Normal, async () =>
            {
                // Only handle this event if this page is currently being displayed
                if (args.Property == SystemMediaTransportControlsProperty.SoundLevel && Frame.CurrentSourcePageType == typeof(MainPage))
                {
                    // Check to see if the app is being muted. If so, it is being minimized.
                    // Otherwise if it is not initialized, it is being brought into focus.
                    if (sender.SoundLevel == SoundLevel.Muted)
                    {
                        await CleanupCameraAsync();
                    }
                    else if (!_isInitialized)
                    {
                        await InitializeCameraAsync();
                    }
                }
            });
        }

        /// <summary>
        /// This event will fire when the page is rotated
        /// </summary>
        /// <param name="sender">The event source.</param>
        /// <param name="args">The event data.</param>
        private async void DisplayInformation_OrientationChanged(DisplayInformation sender, object args)
        {
            _displayOrientation = sender.CurrentOrientation;

            if (_isPreviewing)
            {
                await SetPreviewRotationAsync();
            }
        }

        private async void GetPreviewFrameButton_Click(object sender, RoutedEventArgs e)
        {
            // If preview is not running, no preview frames can be acquired
            if (!_isPreviewing) return;

            if ((ShowFrameCheckBox.IsChecked == true) || (SaveFrameCheckBox.IsChecked == true))
            {
                await GetPreviewFrameAsSoftwareBitmapAsync();
            }
            else
            {
                await GetPreviewFrameAsD3DSurfaceAsync();
            }
        }

        private async void MediaCapture_Failed(MediaCapture sender, MediaCaptureFailedEventArgs errorEventArgs)
        {
            Debug.WriteLine("MediaCapture_Failed: (0x{0:X}) {1}", errorEventArgs.Code, errorEventArgs.Message);

            await CleanupCameraAsync();

            await Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () => GetPreviewFrameButton.IsEnabled = _isPreviewing);
        }

        #endregion Event handlers


        #region MediaCapture methods

        /// <summary>
        /// Initializes the MediaCapture, registers events, gets camera device information for mirroring and rotating, and starts preview
        /// </summary>
        /// <returns></returns>
        private async Task InitializeCameraAsync()
        {
            Debug.WriteLine("InitializeCameraAsync");

            if (_mediaCapture == null)
            {
                // Attempt to get the back camera if one is available, but use any camera device if not
                //var cameraDevice = await FindCameraDeviceByPanelAsync(Windows.Devices.Enumeration.Panel.Back);

                // Fujimaki Add カメラデバイスの選択処理
                DeviceInformationCollection cameraDevices = await DeviceInformation.FindAllAsync(DeviceClass.VideoCapture);
                DeviceInformation cameraDevice = null;
                for (int i = 0; i < cameraDevices.Count(); i++)
                {
                    if (cameraDevices.ElementAt(i).Name != _cameraDeviceNames[_cameraID])
                    {
                        continue;
                    }
                    cameraDevice = cameraDevices.ElementAt(i);
                }
                // Fujimaki Add End カメラデバイスの選択処理

                if (cameraDevice == null)
                {
                    Debug.WriteLine("No camera device found!");
                    return;
                }

                // Create MediaCapture and its settings
                _mediaCapture = new MediaCapture();

                // Register for a notification when something goes wrong
                _mediaCapture.Failed += MediaCapture_Failed;

                var settings = new MediaCaptureInitializationSettings { VideoDeviceId = cameraDevice.Id };

                // Initialize MediaCapture
                try
                {
                    await _mediaCapture.InitializeAsync(settings);
                    _isInitialized = true;
                }
                catch (UnauthorizedAccessException)
                {
                    Debug.WriteLine("The app was denied access to the camera");
                }

                // If initialization succeeded, start the preview
                if (_isInitialized)
                {
                    // Figure out where the camera is located
                    if (cameraDevice.EnclosureLocation == null || cameraDevice.EnclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Unknown)
                    {
                        // No information on the location of the camera, assume it's an external camera, not integrated on the device
                        _externalCamera = true;
                    }
                    else
                    {
                        // Camera is fixed on the device
                        _externalCamera = false;

                        // Only mirror the preview if the camera is on the front panel
                        _mirroringPreview = (cameraDevice.EnclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Front);
                    }

                    await StartPreviewAsync();

                    var picturesLibrary = await StorageLibrary.GetLibraryAsync(KnownLibraryId.Pictures);
                    // Fall back to the local app storage if the Pictures Library is not available
                    _captureFolder = picturesLibrary.SaveFolder ?? ApplicationData.Current.LocalFolder;
                }
            }
        }

        /// <summary>
        /// Starts the preview and adjusts it for for rotation and mirroring after making a request to keep the screen on and unlocks the UI
        /// </summary>
        /// <returns></returns>
        private async Task StartPreviewAsync()
        {
            Debug.WriteLine("StartPreviewAsync");

            // Prevent the device from sleeping while the preview is running
            _displayRequest.RequestActive();

            // Register to listen for media property changes
            _systemMediaControls.PropertyChanged += SystemMediaControls_PropertyChanged;

            // Set the preview source in the UI and mirror it if necessary
            PreviewControl.Source = _mediaCapture;
            PreviewControl.FlowDirection = _mirroringPreview ? FlowDirection.RightToLeft : FlowDirection.LeftToRight;

            // Start the preview
            await _mediaCapture.StartPreviewAsync();
            _isPreviewing = true;

            // Initialize the preview to the current orientation
            if (_isPreviewing)
            {
                await SetPreviewRotationAsync();
            }

            // Enable / disable the button depending on the preview state
            GetPreviewFrameButton.IsEnabled = _isPreviewing;
        }

        /// <summary>
        /// Gets the current orientation of the UI in relation to the device and applies a corrective rotation to the preview
        /// </summary>
        private async Task SetPreviewRotationAsync()
        {
            // Only need to update the orientation if the camera is mounted on the device
            if (_externalCamera) return;

            // Calculate which way and how far to rotate the preview
            int rotationDegrees = ConvertDisplayOrientationToDegrees(_displayOrientation);

            // The rotation direction needs to be inverted if the preview is being mirrored
            if (_mirroringPreview)
            {
                rotationDegrees = (360 - rotationDegrees) % 360;
            }

            // Add rotation metadata to the preview stream to make sure the aspect ratio / dimensions match when rendering and getting preview frames
            var props = _mediaCapture.VideoDeviceController.GetMediaStreamProperties(MediaStreamType.VideoPreview);
            props.Properties.Add(RotationKey, rotationDegrees);
            await _mediaCapture.SetEncodingPropertiesAsync(MediaStreamType.VideoPreview, props, null);
        }

        /// <summary>
        /// Stops the preview and deactivates a display request, to allow the screen to go into power saving modes, and locks the UI
        /// </summary>
        /// <returns></returns>
        private async Task StopPreviewAsync()
        {
            _isPreviewing = false;
            await _mediaCapture.StopPreviewAsync();

            // Use the dispatcher because this method is sometimes called from non-UI threads
            await Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                PreviewControl.Source = null;

                // Allow the device to sleep now that the preview is stopped
                _displayRequest.RequestRelease();

                GetPreviewFrameButton.IsEnabled = _isPreviewing;
            });
        }

        // Fujimaki Add
        private async void _timer_Tick(object sender, object e)
        {
            //await GetPreviewFrameAsSoftwareBitmapAsync();
            try
            {
                await PlaySoundAsync();
            }
            catch (Exception excep)
            {
                this.ValueText.Text = excep.Message;
            }
        }

        // Fujimaki Add
        private async Task PlaySoundAsync()
        {
            // Get information about the preview
            var previewProperties = _mediaCapture.VideoDeviceController.GetMediaStreamProperties(MediaStreamType.VideoPreview) as VideoEncodingProperties;

            // Create the video frame to request a SoftwareBitmap preview frame
            var videoFrame = new VideoFrame(BitmapPixelFormat.Bgra8, (int)previewProperties.Width, (int)previewProperties.Height);

            // Capture the preview frame
            using (var currentFrame = await _mediaCapture.GetPreviewFrameAsync(videoFrame))
            {
                // Collect the resulting frame
                SoftwareBitmap previewFrame = currentFrame.SoftwareBitmap;
                double[,] hues = null;  // 色合いのヒストグラムデータ
                GetHues(previewFrame, out hues);
                double[,] noiseRemovedHues = null;
                RemoveNoise(hues, out noiseRemovedHues);
                double[] filter = new double[_smoothFilterLength]; // ヒストグラムの平滑化に用いるフィルタ
                for (int i = 0; i < filter.Length; i++) filter[i] = 1.0;
                SmoothHist(noiseRemovedHues, filter);   // ヒストグラムを平滑化する
                int[,] huePeaks = null;  // 色合いのピークデータ
                GetHuePeaks(noiseRemovedHues, out huePeaks);
                ResetPlayEnable();
                for (int i = 0; i < huePeaks.GetLength(0); i++)
                {
                    for (int j = 0; j < huePeaks.GetLength(1); j++)
                    {
                        try
                        {
                            if (huePeaks[i, j] > 0)
                            {
                                string colorName = "";
                                int gakkiId = -1;
                                GetColorNameFromHueValue((double)i, out colorName, out gakkiId);
                                _playEnable[gakkiId, j] = true;
                            }
                        }
                        catch (Exception excep)
                        {
                            int a = 0;
                        }
                    }
                }

                string txt = "";
                try
                {
                    for (int i = 0; i < _playEnable.GetLength(0); i++)
                    {
                        for (int j = 0; j < _playEnable.GetLength(1); j++)
                        {
                            if (_playEnable[i, j])
                            {
                                _soundPlayer[i, j].Play();
                                txt += i.ToString() + ":" + j.ToString() + "|";
                            }
                        }
                    }
                }
                catch (Exception excep)
                {
                    int a = 0;
                }
                this.ValueText.Text = txt;
            }
        }

        /// <summary>
        /// Gets the current preview frame as a SoftwareBitmap, displays its properties in a TextBlock, and can optionally display the image
        /// in the UI and/or save it to disk as a jpg
        /// </summary>
        /// <returns></returns>
        private async Task GetPreviewFrameAsSoftwareBitmapAsync()
        {
            // Get information about the preview
            var previewProperties = _mediaCapture.VideoDeviceController.GetMediaStreamProperties(MediaStreamType.VideoPreview) as VideoEncodingProperties;

            // Create the video frame to request a SoftwareBitmap preview frame
            var videoFrame = new VideoFrame(BitmapPixelFormat.Bgra8, (int)previewProperties.Width, (int)previewProperties.Height);

            // Capture the preview frame
            using (var currentFrame = await _mediaCapture.GetPreviewFrameAsync(videoFrame))
            {
                // Collect the resulting frame
                SoftwareBitmap previewFrame = currentFrame.SoftwareBitmap;

                // Show the frame information
                FrameInfoTextBlock.Text = String.Format("{0}x{1} {2}", previewFrame.PixelWidth, previewFrame.PixelHeight, previewFrame.BitmapPixelFormat);

                // Add a simple green filter effect to the SoftwareBitmap
                if (GreenEffectCheckBox.IsChecked == true)
                {
                    ApplyGreenFilter(previewFrame);
                }

                // Show the frame (as is, no rotation is being applied)
                if (ShowFrameCheckBox.IsChecked == true)
                {
                    // Create a SoftwareBitmapSource to display the SoftwareBitmap to the user
                    var sbSource = new SoftwareBitmapSource();
                    await sbSource.SetBitmapAsync(previewFrame);

                    // Display it in the Image control
                    PreviewFrameImage.Source = sbSource;
                }

                // Save the frame (as is, no rotation is being applied)
                if (SaveFrameCheckBox.IsChecked == true)
                {
                    var file = await _captureFolder.CreateFileAsync("PreviewFrame.jpg", CreationCollisionOption.GenerateUniqueName);

                    Debug.WriteLine("Saving preview frame to " + file.Path);

                    await SaveSoftwareBitmapAsync(previewFrame, file);
                }
            }
        }

        /// <summary>
        /// Gets the current preview frame as a Direct3DSurface and displays its properties in a TextBlock
        /// </summary>
        /// <returns></returns>
        private async Task GetPreviewFrameAsD3DSurfaceAsync()
        {
            // Capture the preview frame as a D3D surface
            using (var currentFrame = await _mediaCapture.GetPreviewFrameAsync())
            {
                // Check that the Direct3DSurface isn't null. It's possible that the device does not support getting the frame
                // as a D3D surface
                if (currentFrame.Direct3DSurface != null)
                {
                    // Collect the resulting frame
                    var surface = currentFrame.Direct3DSurface;

                    // Show the frame information
                    FrameInfoTextBlock.Text = String.Format("{0}x{1} {2}", surface.Description.Width, surface.Description.Height, surface.Description.Format);
                }
                else // Fall back to software bitmap
                {
                    // Collect the resulting frame
                    SoftwareBitmap previewFrame = currentFrame.SoftwareBitmap;

                    // Show the frame information
                    FrameInfoTextBlock.Text = String.Format("{0}x{1} {2}", previewFrame.PixelWidth, previewFrame.PixelHeight, previewFrame.BitmapPixelFormat);
                }

                // Clear the image
                PreviewFrameImage.Source = null;
            }
        }

        /// <summary>
        /// Cleans up the camera resources (after stopping the preview if necessary) and unregisters from MediaCapture events
        /// </summary>
        /// <returns></returns>
        private async Task CleanupCameraAsync()
        {
            if (_isInitialized)
            {
                if (_isPreviewing)
                {
                    // The call to stop the preview is included here for completeness, but can be
                    // safely removed if a call to MediaCapture.Dispose() is being made later,
                    // as the preview will be automatically stopped at that point
                    await StopPreviewAsync();
                }

                _isInitialized = false;
            }

            if (_mediaCapture != null)
            {
                _mediaCapture.Failed -= MediaCapture_Failed;
                _mediaCapture.Dispose();
                _mediaCapture = null;
            }
        }

        #endregion MediaCapture methods


        #region Helper functions

        /// <summary>
        /// Queries the available video capture devices to try and find one mounted on the desired panel
        /// </summary>
        /// <param name="desiredPanel">The panel on the device that the desired camera is mounted on</param>
        /// <returns>A DeviceInformation instance with a reference to the camera mounted on the desired panel if available,
        ///          any other camera if not, or null if no camera is available.</returns>
        private static async Task<DeviceInformation> FindCameraDeviceByPanelAsync(Windows.Devices.Enumeration.Panel desiredPanel)
        {
            // Get available devices for capturing pictures
            var allVideoDevices = await DeviceInformation.FindAllAsync(DeviceClass.VideoCapture);

            // Get the desired camera by panel
            DeviceInformation desiredDevice = allVideoDevices.FirstOrDefault(x => x.EnclosureLocation != null && x.EnclosureLocation.Panel == desiredPanel);

            // If there is no device mounted on the desired panel, return the first device found
            return desiredDevice ?? allVideoDevices.FirstOrDefault();
        }

        /// <summary>
        /// Converts the given orientation of the app on the screen to the corresponding rotation in degrees
        /// </summary>
        /// <param name="orientation">The orientation of the app on the screen</param>
        /// <returns>An orientation in degrees</returns>
        private static int ConvertDisplayOrientationToDegrees(DisplayOrientations orientation)
        {
            switch (orientation)
            {
                case DisplayOrientations.Portrait:
                    return 90;
                case DisplayOrientations.LandscapeFlipped:
                    return 180;
                case DisplayOrientations.PortraitFlipped:
                    return 270;
                case DisplayOrientations.Landscape:
                default:
                    return 0;
            }
        }

        /// <summary>
        /// Saves a SoftwareBitmap to the specified StorageFile
        /// </summary>
        /// <param name="bitmap">SoftwareBitmap to save</param>
        /// <param name="file">Target StorageFile to save to</param>
        /// <returns></returns>
        private static async Task SaveSoftwareBitmapAsync(SoftwareBitmap bitmap, StorageFile file)
        {
            using (var outputStream = await file.OpenAsync(FileAccessMode.ReadWrite))
            {
                var encoder = await BitmapEncoder.CreateAsync(BitmapEncoder.JpegEncoderId, outputStream);

                // Grab the data from the SoftwareBitmap
                encoder.SetSoftwareBitmap(bitmap);
                await encoder.FlushAsync();
            }
        }

        /// <summary>
        /// Applies a basic effect to a Bgra8 SoftwareBitmap in-place
        /// </summary>
        /// <param name="bitmap">SoftwareBitmap that will receive the effect</param>
        private unsafe void ApplyGreenFilter(SoftwareBitmap bitmap)
        {
            // Effect is hard-coded to operate on BGRA8 format only
            if (bitmap.BitmapPixelFormat == BitmapPixelFormat.Bgra8)
            {
                // In BGRA8 format, each pixel is defined by 4 bytes
                const int BYTES_PER_PIXEL = 4;

                using (var buffer = bitmap.LockBuffer(BitmapBufferAccessMode.ReadWrite))
                using (var reference = buffer.CreateReference())
                {
                    if (reference is IMemoryBufferByteAccess)
                    {
                        // Get a pointer to the pixel buffer
                        byte* data;
                        uint capacity;
                        ((IMemoryBufferByteAccess)reference).GetBuffer(out data, out capacity);

                        // Get information about the BitmapBuffer
                        var desc = buffer.GetPlaneDescription(0);

                        // Iterate over all pixels
                        for (uint row = 0; row < desc.Height; row++)
                        {
                            for (uint col = 0; col < desc.Width; col++)
                            {
                                // Index of the current pixel in the buffer (defined by the next 4 bytes, BGRA8)
                                var currPixel = desc.StartIndex + desc.Stride * row + BYTES_PER_PIXEL * col;

                                // Read the current pixel information into b,g,r channels (leave out alpha channel)
                                var b = data[currPixel + 0]; // Blue
                                var g = data[currPixel + 1]; // Green
                                var r = data[currPixel + 2]; // Red

                                // Boost the green channel, leave the other two untouched
                                data[currPixel + 0] = b;
                                data[currPixel + 1] = (byte)Math.Min(g + 80, 255);
                                data[currPixel + 2] = r;
                            }
                        }
                    }
                }
            }
        }

        // Fujimaki Add
        private unsafe int GetAverageBrightness(SoftwareBitmap bitmap)
        {
            int output = 0;

            // In BGRA8 format, each pixel is defined by 4 bytes
            const int BYTES_PER_PIXEL = 4;

            using (var buffer = bitmap.LockBuffer(BitmapBufferAccessMode.ReadWrite))
            using (var reference = buffer.CreateReference())
            {
                if (reference is IMemoryBufferByteAccess)
                {
                    // Get a pointer to the pixel buffer
                    byte* data;
                    uint capacity;
                    ((IMemoryBufferByteAccess)reference).GetBuffer(out data, out capacity);

                    // Get information about the BitmapBuffer
                    var desc = buffer.GetPlaneDescription(0);

                    // Iterate over all pixels
                    for (uint row = 0; row < desc.Height; row++)
                    {
                        for (uint col = 0; col < desc.Width; col++)
                        {
                            // Index of the current pixel in the buffer (defined by the next 4 bytes, BGRA8)
                            var currPixel = desc.StartIndex + desc.Stride * row + BYTES_PER_PIXEL * col;

                            // Read the current pixel information into b,g,r channels (leave out alpha channel)
                            var b = data[currPixel + 0]; // Blue
                            var g = data[currPixel + 1]; // Green
                            var r = data[currPixel + 2]; // Red

                            output += b + g + r;
                        }
                    }

                    output = output / (3 * desc.Height * desc.Width);
                }
            }

            return output;
        }

        private unsafe void GetHues(SoftwareBitmap bitmap, out double[,] hues)
        {
            // In BGRA8 format, each pixel is defined by 4 bytes
            const int BYTES_PER_PIXEL = 4;

            using (var buffer = bitmap.LockBuffer(BitmapBufferAccessMode.ReadWrite))
            using (var reference = buffer.CreateReference())
            {
                if (reference is IMemoryBufferByteAccess)
                {
                    hues = new double[360, _numSoundPerGakki];
                    for (int i = 0; i < 360; i++)
                    {
                        for (int j = 0; j < _numSoundPerGakki; j++)
                        {
                            hues[i, j] = 0;
                        }
                    }

                    // Get a pointer to the pixel buffer
                    byte* data;
                    uint capacity;
                    ((IMemoryBufferByteAccess)reference).GetBuffer(out data, out capacity);

                    // Get information about the BitmapBuffer
                    var desc = buffer.GetPlaneDescription(0);

                    // Iterate over all pixels
                    for (uint row = 0; row < desc.Height; row++)
                    {
                        for (uint col = 0; col < desc.Width; col++)
                        {
                            // Index of the current pixel in the buffer (defined by the next 4 bytes, BGRA8)
                            var currPixel = desc.StartIndex + desc.Stride * row + BYTES_PER_PIXEL * col;

                            // 色合いの値を取得する。
                            HSVColor hsv = Models.ColorHelper.RGBtoHSV(
                                Color.FromArgb(
                                    255,
                                    data[currPixel + 2],    // Red 
                                    data[currPixel + 1],    // Green 
                                    data[currPixel + 0]     // Blue
                                    )
                                );
                            double hue = hsv.H;
                            double brightness = hsv.V;
                            try
                            {
                                hues[(int)hue, (int)(brightness * 7.0 / 255.0)] += 1.0;
                            }
                            catch (Exception excep)
                            {
                                int a = 0;
                            }
                        }
                    }
                }
                else
                {
                    hues = null;
                }
            }
        }

        /// <summary>
        /// 異常値などのノイズ成分を取り除く。
        /// </summary>
        /// <param name="hues">360x8 double array</param>
        /// <param name="output">360x8 double array</param>
        private void RemoveNoise(double[,] hues, out double[,] output)
        {
            output = new double[hues.GetLength(0), hues.GetLength(1)];

            // 閾値よりも低いカウント数のものは除去する。
            try
            {
                for (int i = 0; i < hues.GetLength(0); i++)
                {
                    for (int j = 0; j < hues.GetLength(1); j++)
                    {
                        output[i, j] = (hues[i, j] < _hueThreshold ? 0.0 : hues[i, j]);
                    }
                }
            }
            catch (Exception excep)
            {
                int a = 0;
            }

            // 両隣りの色相のカウント数がゼロのものは除去する。
            for (int j = 0; j < hues.GetLength(1); j++)
            {
                try
                {
                    if (
                        (hues[hues.GetLength(0) - 1, j] < 0.001) &&
                        (hues[0, j] > 0.0) &&
                        (hues[1, j] < 0.001)
                        )
                    {
                        output[0, j] = 0.0;
                    }
                }
                catch (Exception excep)
                {
                    int a = 0;
                }
                try
                {
                    for (int i = 1; i < hues.GetLength(0) - 1; i++)
                    {
                        if (
                            (hues[i - 1, j] < 0.001) &&
                            (hues[i, j] > 0.0) &&
                            (hues[i + 1, j] < 0.001)
                            )
                        {
                            output[i, j] = 0.0;
                        }
                    }
                }
                catch (Exception excep)
                {
                    int a = 0;
                }
                try
                {
                    if (
                        (hues[hues.GetLength(0) - 2, j] < 0.001) &&
                        (hues[hues.GetLength(0) - 1, j] > 0.0) &&
                        (hues[0, j] < 0.001)
                        )
                    {
                        output[hues.GetLength(0) - 1, j] = 0.0;
                    }
                }
                catch (Exception excep)
                {
                    int a = 0;
                }
            }
        }

        private void GetHuePeaks(double[,] hues, out int[,] huePeaks)
        {
            huePeaks = new int[hues.GetLength(0), hues.GetLength(1)];
            for (int i = 0; i < hues.GetLength(0); i++)
            {
                for (int j = 0; j < hues.GetLength(1); j++)
                {
                    huePeaks[i, j] = 0;
                }
            }

            double[] tmpHues = new double[hues.GetLength(0) + 2];
            for (int j = 0; j < hues.GetLength(1); j++)
            {
                tmpHues[0] = hues[hues.GetLength(0) - 1, j];
                for (int i = 0; i < hues.GetLength(0); i++)
                {
                    tmpHues[i + 1] = hues[i, j];
                }
                tmpHues[tmpHues.Length - 1] = hues[0, j];

                for (int i = 0; i < hues.GetLength(0); i++)
                {
                    if (
                        (tmpHues[i + 1] > _hueThreshold) &&
                        (tmpHues[i] < tmpHues[i + 1]) &&
                        (tmpHues[i + 1] > tmpHues[i + 2])
                        )
                    {
                        huePeaks[i, j] = 1;
                    }
                }
            }
        }

        private void SmoothHist(double[,] hues, double[] filter)
        {
            if (filter.Length % 2 == 0)
            {
                throw new ArgumentException("フィルタに指定する配列の要素数は奇数となるようにしてください。");
            }

            double[] tmpHues = new double[hues.GetLength(0) + filter.Length - 1];
            int arrayShift = filter.Length / 2;

            for (int j = 0; j < hues.GetLength(1); j++)
            {
                for (int i = 0; i < arrayShift; i++)
                {
                    tmpHues[i] = hues[hues.GetLength(0) - arrayShift + i, j];
                    tmpHues[tmpHues.Length - arrayShift + i] = hues[i, j];
                }

                for (int i = 0; i < hues.GetLength(0); i++)
                {
                    tmpHues[i + arrayShift] = hues[i, j];
                }

                for (int i = 0; i < hues.GetLength(0); i++)
                {
                    double v = 0.0;
                    for (int k = 0; k < filter.Length; k++)
                    {
                        v += tmpHues[i + k] * filter[k];
                    }
                    v /= filter.Length;
                    hues[i, j] = v;
                }
            }
        }

        private void ResetPlayEnable()
        {
            for (int i = 0; i < _playEnable.GetLength(0); i++)
            {
                for (int j = 0; j < _playEnable.GetLength(1); j++)
                {
                    _playEnable[i, j] = false;
                }
            }
        }

        private void GetColorNameFromHueValue(double hue, out string colorName, out int gakkiId)
        {
            if (hue < 0.0)
            {
                throw new ArgumentException();
            }

            colorName = "";
            gakkiId = -1;

            if (hue < 32.0)
            {
                colorName = "赤";
                gakkiId = 0;
            }
            else if (hue < 50.0)
            {
                colorName = "橙";
                gakkiId = 1;
            }
            else if (hue < 64.0)
            {
                colorName = "黄";
                gakkiId = 1;
            }
            else if (hue < 151.0)
            {
                colorName = "緑";
                gakkiId = 2;
            }
            else if (hue < 194.0)
            {
                colorName = "水色";
                gakkiId = 3;
            }
            else if (hue < 266.0)
            {
                colorName = "青";
                gakkiId = 3;
            }
            else if (hue < 295.0)
            {
                colorName = "紫";
                gakkiId = 0;
            }
            else
            {
                colorName = "赤";
                gakkiId = 0;
            }
        }

        #endregion Helper functions 
    }
}