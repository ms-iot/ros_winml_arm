using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;

using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.Messages.Standard;
using sensor_msgs = RosSharp.RosBridgeClient.Messages.Sensor;
using std_srvs = RosSharp.RosBridgeClient.Services.Standard;
using rosapi = RosSharp.RosBridgeClient.Services.RosApi;
using rosgraph = RosSharp.RosBridgeClient.Messages.rosgraph;

using Windows.UI.Xaml.Media.Imaging;
using Windows.Storage.Streams;
using Windows.UI.Core;
using Windows.Graphics.Imaging;
using System.Runtime.InteropServices;
using Windows.ApplicationModel.Core;
// The Blank Page item template is documented at https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace embedded_world
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        private RosControllerViewModel vm;

        public MainPage()
        {
            this.InitializeComponent();
            vm = (RosControllerViewModel)this.DataContext;

            btnClearLog.Click += delegate (object sender, RoutedEventArgs args)
            {
                vm.ClearLogEntries();
            };

            btnHome.Click += delegate (object sender, RoutedEventArgs args)
            {
                vm.Home_Click(sender, args);
            };

            btnPickUp.Click += delegate (object sender, RoutedEventArgs args)
            {
                vm.Pickup_Click(sender, args);
            };

            btnPutDown.Click += delegate (object sender, RoutedEventArgs args)
            {
                vm.Place_Click(sender, args);
            };

            btnConnect.Click += delegate (object sender, RoutedEventArgs args)
            {
                vm.Connect_Click(sender, args);
            };

        }

        private void Page_Loaded(object sender, RoutedEventArgs e)
        {
            vm.connect();
        }
    }


    public class RosControllerViewModel : Bindable
    {
        [ComImport]
        [Guid("5B0D3235-4DBA-4D44-865E-8F1D0E4FD04D")]
        [InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        private unsafe interface IMemoryBufferByteAccess
        {
            void GetBuffer(out byte* buffer, out uint capacity);
        }

        public ObservableCollection<RosCSharpLogItem> LogItems { get; set; } = new ObservableCollection<RosCSharpLogItem>();
        public RosTaskStatus CurrentTask { get; set; } = new RosTaskStatus() { LogItemDate = DateTimeOffset.Now, Title = "initializing", Message = "..." };

        public RosConnectionStatus ControllerStatus { get; set; } = new RosConnectionStatus() { CurrentState = RosConnectionState.Connecting };
        public RosConnectionStatus RosStatus { get; set; } = new RosConnectionStatus() { CurrentState = RosConnectionState.Connecting };
        public RosConnectionStatus KinectStatus { get; set; } = new RosConnectionStatus() { CurrentState = RosConnectionState.Connecting };
        public SoftwareBitmapSource MLView { get; set; } = new SoftwareBitmapSource();
        public string Confidence { get; set; } = "";

        private const int MaxLogEntries = 20;

        string imageSubId;
        string logSubId;
        string commandPubId;
        string trackedSubId;

        static readonly string uri = "ws://127.0.0.1:9090";

        DispatcherTimer dispatcherTimer = new DispatcherTimer();

        RosSocket rosSocket;
        RosSharp.RosBridgeClient.Protocols.WebSocketUWPProtocol rosWebSocketProtocol;


        public RosControllerViewModel()
        {
            UpdateCurrentTask("Initializing The Connections", "");

            //SetControllerStatus(RosConnectionState.Connecting);
            SetRosStatus(RosConnectionState.Connected);
            SetKinectStatus(RosConnectionState.Connected);
        }


        internal void connect()
        {
            if (rosSocket != null)
            {
                rosSocket.Close();
                rosSocket = null;
            }

            rosWebSocketProtocol = new RosSharp.RosBridgeClient.Protocols.WebSocketUWPProtocol(uri);

            // Todo refactor the async connector.
            rosSocket = new RosSocket(rosWebSocketProtocol);
            imageSubId = rosSocket.Subscribe<sensor_msgs.Image>("/tracked_objects/image", SubscriptionHandler);
            logSubId = rosSocket.Subscribe<rosgraph.Log>("/rosout", LogSubscriptionHandler);
            trackedSubId = rosSocket.Subscribe<DetectedObjectPose>("/detected_object", DetectedSubscriptionHandler);
            commandPubId = rosSocket.Advertise<std_msgs.RosInt32>("/goto");

            dispatcherTimer.Tick += DispatcherTimer_Tick;
            dispatcherTimer.Interval = new TimeSpan(0, 0, 3);
            dispatcherTimer.Start();

            UpdateCurrentTask("Connected", "");

        }

        private void DispatcherTimer_Tick(object sender, object e)
        {
            var ignore = CoreApplication.MainView.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                Confidence = String.Format("Object not found");
                NotifyPropertyChanged("Confidence");
            });
        }

        public void SetControllerStatus(RosConnectionState newState)
        {
            ControllerStatus.SetNewState(newState);
        }

        public void SetRosStatus(RosConnectionState newState)
        {
            RosStatus.SetNewState(newState);
        }

        public void SetKinectStatus(RosConnectionState newState)
        {
            KinectStatus.SetNewState(newState);
        }

        public void UpdateCurrentTask(string title, string message)
        {
            CurrentTask.UpdateStatus(title, message);
        }


        internal void Home_Click(object sender, RoutedEventArgs e)
        {
            if (rosSocket != null)
            {
                std_msgs.RosInt32 command = new std_msgs.RosInt32(0);
                rosSocket.Publish(commandPubId, command);
                UpdateCurrentTask("Going Home", "");
            }
        }

        internal void Pickup_Click(object sender, RoutedEventArgs e)
        {
            if (rosSocket != null)
            {
                std_msgs.RosInt32 command = new std_msgs.RosInt32(1);
                rosSocket.Publish(commandPubId, command);
                UpdateCurrentTask("Pickup", "");
            }
        }

        internal void Place_Click(object sender, RoutedEventArgs e)
        {
            if (rosSocket != null)
            {
                std_msgs.RosInt32 command = new std_msgs.RosInt32(2);
                rosSocket.Publish(commandPubId, command);
                UpdateCurrentTask("Place", "");
            }
        }

        internal void Auto_Click(object sender, RoutedEventArgs e)
        {
            if (rosSocket != null)
            {
                std_msgs.RosInt32 command = new std_msgs.RosInt32(3);
                rosSocket.Publish(commandPubId, command);
                UpdateCurrentTask("Auto", "");
            }
        }

        internal void Connect_Click(object sender, RoutedEventArgs e)
        {
            connect();
        }

        public void ClearLogEntries()
        {
            LogItems.Clear();
        }

        public void InsertLogEntry(string itemText)
        {
            InsertLogEntry(new RosCSharpLogItem() { LogItemDate = DateTimeOffset.Now, Message = itemText });
        }

        public void InsertLogEntry(RosCSharpLogItem newItem)
        {
            //Insert Item at top
            if (LogItems.Count > 0)
                LogItems.Insert(0, newItem);
            else
                LogItems.Add(newItem);

            //cleanup items so there's no more than the MaxLogEntries value
            while (LogItems.Count > MaxLogEntries)
                LogItems.RemoveAt(LogItems.Count - 1);

        }

        private void LogSubscriptionHandler(rosgraph.Log message)
        {
            var ignore = CoreApplication.MainView.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                InsertLogEntry(new RosCSharpLogItem() { LogItemDate = DateTimeOffset.Now, Message = message.msg });
            });
        }

        private void DetectedSubscriptionHandler(DetectedObjectPose message)
        {
            var ignore = CoreApplication.MainView.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                Confidence = String.Format("Confidence: {0:P2}.", message.confidence);
                dispatcherTimer.Stop();
                dispatcherTimer.Start();
                NotifyPropertyChanged("Confidence");

            });
        }

        private void SubscriptionHandler(sensor_msgs.Image message)
        {
            var ignore = CoreApplication.MainView.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, async () =>
            {
                var softwareBitmap = new SoftwareBitmap(BitmapPixelFormat.Bgra8, message.width, message.height, BitmapAlphaMode.Ignore);

                unsafe
                {
                    using (BitmapBuffer buffer = softwareBitmap.LockBuffer(BitmapBufferAccessMode.Write))
                    {
                        using (var reference = buffer.CreateReference())
                        {
                            byte* dataInBytes;
                            uint capacity;
                            ((IMemoryBufferByteAccess)reference).GetBuffer(out dataInBytes, out capacity);

                            // Fill-in the BGRA plane
                            BitmapPlaneDescription bufferLayout = buffer.GetPlaneDescription(0);
                            for (int y = 0; y < bufferLayout.Height; y++)
                            {
                                uint xoffset = 0;
                                for (int x = 0; x < bufferLayout.Stride; x += 4)
                                {
                                    dataInBytes[bufferLayout.StartIndex + bufferLayout.Stride * y + x + 0] = message.data[message.step * y + xoffset + 2];
                                    dataInBytes[bufferLayout.StartIndex + bufferLayout.Stride * y + x + 1] = message.data[message.step * y + xoffset + 1];
                                    dataInBytes[bufferLayout.StartIndex + bufferLayout.Stride * y + x + 2] = message.data[message.step * y + xoffset + 0];
                                    dataInBytes[bufferLayout.StartIndex + bufferLayout.Stride * y + x + 3] = (byte)255;

                                    xoffset += 3;
                                }
                            }
                        }
                    }
                }

                await MLView.SetBitmapAsync(softwareBitmap);

                NotifyPropertyChanged("MLView");
            });
        }



    }

    public class RosConnectionStatus : Bindable
    {
        public RosConnectionState CurrentState { get; set; } = RosConnectionState.Connecting;

        public Visibility ShowConnecting =>
            CurrentState == RosConnectionState.Connecting ? Visibility.Visible : Visibility.Collapsed;

        public Visibility ShowConnected =>
            CurrentState == RosConnectionState.Connected ? Visibility.Visible : Visibility.Collapsed;

        public Visibility ShowFailed =>
            CurrentState == RosConnectionState.Failed ? Visibility.Visible : Visibility.Collapsed;

        public void SetNewState(RosConnectionState newState)
        {
            CurrentState = newState;
            NotifyPropertyChanged("ShowConnecting");
            NotifyPropertyChanged("ShowConnected");
            NotifyPropertyChanged("ShowFailed");
        }
    }

    public enum RosConnectionState
    {
        Connecting,
        Connected,
        Failed
    }


    public class RosCSharpLogItem
    {
        public DateTimeOffset LogItemDate { get; set; } = DateTimeOffset.Now;
        public string Message { get; set; }
    }

    public class RosTaskStatus : Bindable
    {
        public DateTimeOffset LogItemDate { get; set; } = DateTimeOffset.Now;
        public string Title { get; set; }
        public string Message { get; set; }

        public void UpdateStatus(string title, string message)
        {
            Title = title;
            Message = message;
            NotifyPropertyChanged("Title");
            NotifyPropertyChanged("Message");
        }
    }

    public class Bindable : INotifyPropertyChanged
    {

        public event PropertyChangedEventHandler PropertyChanged;

        // This method is called by the Set accessor of each property. 
        // The CallerMemberName attribute that is applied to the optional propertyName 
        // parameter causes the property name of the caller to be substituted as an argument. 
        protected void NotifyPropertyChanged([CallerMemberName] String propertyName = "")
        {
            if (1 == 1) //(PropertyChanged != null)
            {
                if (PropertyChanged == null) return;
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }

    }

}
