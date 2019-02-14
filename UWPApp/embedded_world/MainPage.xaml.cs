using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
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
// The Blank Page item template is documented at https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace embedded_world
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        [ComImport]
        [Guid("5B0D3235-4DBA-4D44-865E-8F1D0E4FD04D")]
        [InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        private unsafe interface IMemoryBufferByteAccess
        {
            void GetBuffer(out byte* buffer, out uint capacity);
        }

        static readonly string uri = "ws://127.0.0.1:9090";

        string imageSubId;
        string logSubId;
        RosSocket rosSocket;
        RosSharp.RosBridgeClient.Protocols.WebSocketUWPProtocol rosWebSocketProtocol;

        public MainPage()
        {
            this.InitializeComponent();
        }

        private void Page_Loaded(object sender, RoutedEventArgs e)
        {
            rosWebSocketProtocol = new RosSharp.RosBridgeClient.Protocols.WebSocketUWPProtocol(uri);
            rosSocket = new RosSocket(rosWebSocketProtocol);

            imageSubId = rosSocket.Subscribe<sensor_msgs.Image>("/tracked_objects/image", SubscriptionHandler);
            logSubId = rosSocket.Subscribe<rosgraph.Log>("/rosout", LogSubscriptionHandler);
        }

        private void LogSubscriptionHandler(rosgraph.Log message)
        {
            var ignore = Dispatcher.RunAsync(CoreDispatcherPriority.Normal, async () =>
            {
                LoggingListBox.Items.Insert(0, new TextBlock() { Text = message.msg});
            });
        }

        private void SubscriptionHandler(sensor_msgs.Image message)
        {
            var ignore = Dispatcher.RunAsync(CoreDispatcherPriority.Normal, async () =>
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
                                for (int x = 0; x < bufferLayout.Stride; x+=4)
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

                var source = new SoftwareBitmapSource();
                await source.SetBitmapAsync(softwareBitmap);

                MLView.Source = source;
            });
        }
    }
}
