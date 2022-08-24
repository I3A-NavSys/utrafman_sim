const express = require('express')
const { spawn } = require('node:child_process')

const app = express()
const port = 3000

var simulation = false
var closing = false

app.get('/deploy/:launchfile', (req, res) => {
    var launchfile = req.params.launchfile

    if (simulation){
		res.status(405).send("Simulation already in progress")
    } else {
        simulation = spawn('roslaunch', ['siam_main', `${launchfile}`]);
        res.send(`Launching simulation with launchfile '${launchfile}' ...`)
    }
})

app.get('/destroy', async (req, res) => {
	if (closing){
		res.status(405).send("Previous simulation is closing")
		return;
	}
    if (simulation) {
    	// simulation.on('close', (code, signal) => {
    	// 	simulation = false
		// 	console.log("Simulation closed.")
    	// })
    	simulation.kill('SIGINT');

	await new Promise(resolve => setTimeout(resolve, 10*1000));
	simulation = false;
	res.send(`Simulation has been finished`)
    } else {
        res.status(405).send("No simulations in progress")
    }
  })

app.get('/reset', (req, res) => {
	spawn('killall', ['-9', 'gzserver'])
	res.send(`killall -9 gzserver has been executed`);
  })

app.listen(port, () => {
  console.log(`The HTTP server to manage SIAMsim simulator has been deployed in port ${port}`)
})
