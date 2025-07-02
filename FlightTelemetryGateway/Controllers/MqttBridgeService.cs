using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.SignalR;
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
        IMqttClient _client;

        public MqttBridgeService(IHubContext<TelemetryHub> hub) => _hub = hub;
        public async Task StartAsync(CancellationToken cancellationToken)
        {
            var factory = new MqttFactory();
            _client = factory.CreateMqttClient();

            _client.ConnectedAsync += async e =>
            {
                await _client.SubscribeAsync(
                    new MqttTopicFilterBuilder()
                    .WithTopic("flight/telemetry")
                    .WithQualityOfServiceLevel(MqttQualityOfServiceLevel.AtLeastOnce)
                    .Build(),
                    cancellationToken);

                _client.ApplicationMessageReceivedAsync += async e =>
               {
                   var msg = Encoding.UTF8.GetString(e.ApplicationMessage.Payload);
                   await _hub.Clients.All.SendAsync("TelemetryUpdate", msg);
               };

                var opts = new MqttClientOptionsBuilder()
                .WithTcpServer("192.168.0.204", 1883)
                .Build();

                await _client.ConnectAsync(opts, cancellationToken);
            };
        }

        public Task StopAsync(CancellationToken cancellationToken)
        {
            return _client.DisconnectAsync();
        }
    }
}
