using FlightTelemetryGateway.Controllers;

var builder = WebApplication.CreateBuilder(args);

builder.WebHost
    .UseUrls("http://0.0.0.0:8080");

// Add services to the container.
builder.Services.AddSignalR();
builder.Services.AddHostedService<MqttBridgeService>();
builder.Services.AddControllers();

// Learn more about configuring OpenAPI at https://aka.ms/aspnet/openapi
builder.Services.AddOpenApi();

var app = builder.Build();

// Configure the HTTP request pipeline.
if (app.Environment.IsDevelopment())
{
    app.MapOpenApi();
}

app.UseHttpsRedirection();

app.UseAuthorization();

app.MapHub<TelemetryHub>("/telemetryHub"); // Map the SignalR hub

app.MapControllers();

app.Run();
