IP = "192.168.2.111";

request = matlab.net.http.RequestMessage;
uri_deploy = matlab.net.URI(sprintf('http://%s:3000/deploy/dronechallenge.launch', IP));
uri_destroy = matlab.net.URI(sprintf('http://%s:3000/destroy', IP));

simulation_scheduled = 1;
stop_simulation = 1;

while simulation_scheduled
    response = send(request, uri_deploy);
    if (response.StatusCode == 200)
        disp(response.Body.Data);
        simulation_scheduled = 0;
    else
        if ~stop_simulation
            disp('Previous simuation is running. Stopping it ... ');
            response = send(request, uri_destroy);
        else
            disp('Stopping simulation.');
            simulation_scheduled = 0;
            response = send(request, uri_destroy);
            disp(response.Body.Data);
        end
    end
end