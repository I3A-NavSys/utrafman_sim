const express = require('express')
const { spawn } = require('node:child_process')

const app = express()
const port = 3000

var simulation = false

app.get('/deploy/:launchfile', (req, res) => {
    var launchfile = req.params.launchfile

    if (simulation){
        res.status(405).send("Process already in progress")
    } else {
        simulation = spawn(`roslaunch siam_main ${launchfile}`);
        res.send(`Launching simulation with launchfile '${launchfile}' ...`)
    }
})

app.get('/destroy', (req, res) => {
    if (simulation) {
        simulation = false
        simulation.kill('SIGINT');
        res.send('Finishing simulation...')
    } else {
        res.status(405).send("No simulations in progress")
    }
  })

app.listen(port, () => {
  console.log(`The HTTP server to manage SIAMsim simulator has been deployed in port ${port}`)
})