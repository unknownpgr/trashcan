var fs = require('fs');

var reqd;
var recd;

/* 
 * before use this module, you have to call init function to initalize directory.
*/
function init(reqDir, recDir) {
    reqd = reqDir;
    recd = recDir;
}

/* 
 * receving file and parse to json with retring additional function
 * name: file name to receive
 * attr: { interval, maxTryCount }
 * callback: (err, json)    (err: if error has been occured, the value of err is true. when can't load or parse file, error will be occured.)
 *                          (json: parsed json element)
 */
var isReceiving = false;
function receiveJSONData(name, attr, callback) {
    // receiving flag for preventing to recall function while this function is running.
    if (isReceiving == true) {
        return;
    }
    isReceiving = true;

    var tmrId;              // for setInterval function
    var tryCount = 0;       // retry count for errors

    function readFileFunction() {
        fs.readFile(recd + '/' + name, 'utf8', (err, data) => {
            if (err) {
                // try to reading file up to maxTryCount
                tryCount++;
                if (tryCount >= attr.maxTryCount) {
                    // error callback
                    callback(true, null);
                    clearInterval(tmrId);
                }
                return;
            }
            
            // Exception handling when json parsing
            var cbErr = false;
            try {
                var jsonData = JSON.parse(data); 
            }
            catch (e) { 
                cbErr = true; 
            }

            // data callback
            callback(cbErr, jsonData);
            clearInterval(tmrId);

            // delete file
            fs.unlink(recd + '/' + name, err => {
                if (err) throw err;
            });
        });    
    }

    // try readFile up to maxTryCount(see readFileFunction)
    tmrId = setInterval(readFileFunction, attr.interval);
}

/* 
 * send json with file
 * name: file name to send
 * callback: (err)    (err: if error has been occured, the value of err is true.)
 */
function sendJSONData(name, json, callback) {
    var cberr = false;
    var data;

    try {
        data = JSON.stringify(json);
    }
    catch (e) {
        cberr = true;
        callback(cberr);
        return;
    }

    fs.writeFile(reqd + '/' + name, data, 'utf8', err => {
        if (err) cberr = true;
    });

    callback(cberr);
}

/* 
 * send string with file
 * name: file name to send
 * callback: (err)    (err: if error has been occured, the value of err is true.)
 */
function sendStringData(name, str, callback) {
    var cberr = false;

    fs.writeFile(reqd + '/' + name, str, 'utf8', err => {
        if (err) cberr = true;
    });

    callback(cberr);
}

module.exports.init = init;
module.exports.receiveJSONData = receiveJSONData;
module.exports.sendJSONData = sendJSONData;
module.exports.sendStringData = sendStringData;
