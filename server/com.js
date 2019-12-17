var fs = require('fs');

var reqd;
var recd;

function init(reqDir, recDir) {
    reqd = reqDir;
    recd = recDir;
}

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
        fs.readFile(recd + '/' + name, (err, data) => {
            if (err) {
                // try to reading file up to maxTryCount
                tryCount++;
                if (tryCount >= attr.maxTryCount) {
                    clearInterval(tmrId);
                }
                callback(true, null);
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
    
            callback(cbErr, jsonData);
            clearInterval(tmrId);
        });    
    }

    // try readFile up to maxTryCount(see readFileFunction)
    tmrId = setInterval(readFileFunction, attr.interval);
}

module.exports.init = init;
module.exports.receiveJSONData = receiveJSONData;
