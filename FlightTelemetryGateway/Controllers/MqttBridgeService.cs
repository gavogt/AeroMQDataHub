using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.SignalR;
using Microsoft.Extensions.Configuration;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Protocol;
using System.Text;

namespace FlightTelemetryGateway.Controllers
{
    [Route("api/[controller]")]
    [ApiController]
    public class MqttBridgeService : IHostedService
    {
        readonly IHubContext<TelemetryHub> _hub;
        readonly IConfiguration _config;
        IMqttClient _client = default!;

        public MqttBridgeService(IHubContext<TelemetryHub> hub, IConfiguration config)
        {
            _hub = hub;
            _config = config;
        }

        public async Task StartAsync(CancellationToken ct)
        {
            var factory = new MqttFactory();
            _client = factory.CreateMqttClient();

            _client.ConnectedAsync += async args =>
            {
                await _client.SubscribeAsync(
                  new MqttTopicFilterBuilder()
                    .WithTopic("flight/telemetry")
                    .WithQualityOfServiceLevel(MqttQualityOfServiceLevel.AtLeastOnce)
                    .Build(),
                  ct);
                await _client.SubscribeAsync(
                  new MqttTopicFilterBuilder()
                    .WithTopic("flight/camera")
                    .WithQualityOfServiceLevel(MqttQualityOfServiceLevel.AtLeastOnce)
                    .Build(),
                  ct);
            };

            _client.ApplicationMessageReceivedAsync += async e =>
            {
                var topic = e.ApplicationMessage.Topic;
                var msg = Encoding.UTF8.GetString(e.ApplicationMessage.Payload);
                if (topic == "flight/telemetry")
                {
                    await _hub.Clients.All.SendAsync("TelemetryUpdate", msg, ct);
                }
                else if (topic == "flight/camera")
                {
                    await _hub.Clients.All.SendAsync("CameraUpdate", msg, ct);
                }
            };

            // Read from appsettings.json
            var tcpServer = _config["Mqtt:TcpServer"] ?? "localhost";
            var port = int.TryParse(_config["Mqtt:Port"], out var p) ? p : 1883;

            var opts = new MqttClientOptionsBuilder()
                .WithTcpServer(tcpServer, port)
                .Build();

            await _client.ConnectAsync(opts, ct);

            await _hub.Clients.All.SendAsync("TelemetryUpdate", "{\"test\":123}", ct);
        }

        public Task StopAsync(CancellationToken ct)
            => _client.DisconnectAsync();
    }
}
