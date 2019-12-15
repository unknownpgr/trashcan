module.exports = (app) => {
    app.get('/current-robot-position', (req, res) => {
        res.json({pos_x: '1.23', pos_y: '3.96'});
        console.log('success to send response');
    });
};