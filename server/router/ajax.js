var com = require('../com');
var path = require('path');

module.exports = (app, dir) => {
    app.get('/map-data', (req, res) => {
        var attr = { interval: 500, maxTryCount: 10 };
        console.log('LOG: response for map-data');

        com.receiveJSONData(dir + '/data/map.json', attr, (err, json) => {
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

        com.sendStringData(dir + '/data/command.req', data, (err) => {
            if (err) {
                res.status(500).send();
                console.log('FAIL: failed to excute go command');
                return;
            }

            res.status(200).send();
            console.log('SUCCESS: success to request go command');
        });
        
    })
};
