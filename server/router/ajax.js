var com = require('../com');
var path = require('path');

var sendPath = '/home/pi/workspace/webInteract/fromServer';
var receivePath = '/home/pi/workspace/webInteract/fromAlgo';
var attr = { interval: 100, maxTryCount: 5 };

module.exports = (app, dir) => {
    app.get('/map-data', (req, res) => {
        console.log('LOG: response for map-data');

        com.receiveJSONData(receivePath + '/map.json', attr, (err, json) => {
            if (err) {
                res.status(500).send();
                console.log('FAIL: failed to response for map-data')
                return;
            }

            res.json(JSON.stringify(json));
            console.log('SUCCESS: success to response for map-data');
        });
    });

    app.post('/cmd/go', (req, res) => {
        console.log('LOG: request to go command');
        console.log(req.body);
        
        var json = req.body;
        var data = `#go ${json.firstNode} ${json.secondNode} ${json.ratio}`;

        com.sendStringData(sendPath + '/command.req', data, (err) => {
            if (err) {
                res.status(500).send();
                console.log('FAIL: failed to excute go command');
                return;
            }

            res.status(200).send();
            console.log('SUCCESS: success to request go command');
        });
        
    })

    app.post('/cmd/startExplore', (req, res) => {
        console.log('LOG: request to start exploring');
        
        var data = '#startExplore'

        com.sendStringData(sendPath + '/command.req', data, (err) => {
            if (err) {
                res.status(500).send();
                console.log('FAIL: failed to excute startExplore command');
                return;
            }

            res.status(200).send();
            console.log('SUCCESS: success to request startExplore command');
        });
    });

    app.get('/status', (req, res) => {
        console.log('LOG: response for status');
    
        var data = '#getStatus'

        var errorResponse = () => {
            res.status(500).send();
            console.log('FAIL: failed to response for status');
        }

        // send request to path finding program
        com.sendStringData(sendPath + '/command.req', data, (err) => {
            if (err) {
                errorResponse();
                return;
            }

            // send response to webbrowser
            com.receiveJSONData(receivePath + '/status.json', attr, (err, json) => {
                if (err) {
                    errorResponse();
                    return;
                }
    
                res.json(JSON.stringify(json));
                console.log('SUCCESS: success to response for status');
            });
        });
    });
};
