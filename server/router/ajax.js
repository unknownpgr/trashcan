var com = require('../com');

module.exports = (app) => {
    app.get('/map-data', (req, res) => {
        com.receiveJSONData(__dirname + "/data/map.json", {}, (err, json) => {

        });

        setTimeout(() => {
            res.json({pos_x: '1.23', pos_y: '3.96'});
            console.log('success to send response');
        }, 3000);
    });
};