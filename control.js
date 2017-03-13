
var socket = null;
var optionToId = {
    "frequency": "frequency",
    "synchronization_burst_us": "sync-burst-us",
    "synchronization_spacing_us": "sync-spacing-us",
    "total_synchronizations": "total-syncs",
    "signal_burst_us": "signal-burst-us",
    "signal_spacing_us": "signal-spacing-us",
    "total_synchronizations": "total-syncs"
};
var forward = reverse = false;
var left = right = false;

function loadFile(file) {
    var reader = new FileReader();
    reader.onload = function (e) {
        try {
            var json = JSON.parse(e.target.result);
        } catch (error) {
            alert("Unable to read JSON: " + error);
            return;
        }
        for (var option in optionToId) {
            if (optionToId.hasOwnProperty(option)) {
                value = json[option];
                if (value !== undefined) {
                    document.getElementById(optionToId[option]).value = value;
                }
            }
        }
    };
    reader.readAsText(file);
}
function handleFileSelect(evt) {
    evt.preventDefault();
    var file = evt.target.files[0];
    loadFile(file);
}
function connect(evt) {
    evt.preventDefault();
    if (socket !== null) {
        return;
    }
    var address = document.getElementById("server-address").value;
    var port = document.getElementById("port").value;
    socket = new WebSocket("ws://" + address + ":" + port);
    socket.onerror = function (evt) {
        alert("Unable to connect to the pi_pcm server; did you start it in TCP mode (--tcp)?");
        socket.close();
    };
    socket.onclose = function (evt) {
        socket = null;
    };
}
function formatControlMessage() {
    var frequency = parseFloat(document.getElementById("frequency").value);
    var deadFrequency = frequency > 40 ? 26.995 : 49.830;

    var command = [{
        "frequency": frequency,
        "dead_frequency": deadFrequency,
        "burst_us": parseFloat(document.getElementById("sync-burst-us").value),
        "spacing_us": parseInt(document.getElementById("sync-spacing-us").value),
        "repeats": parseInt(document.getElementById("total-syncs").value)
    }];

    var forwardCount = forward ? 2 : reverse ? 0 : 1;
    var leftCount = left ? 2 : right ? 0 : 1;
    var burstIdLookup = [
        ["reverse-right", "reverse", "reverse-left"],
        ["", "", ""],  // Not moving
        ["forward-right", "forward", "forward-left"]
    ];
    var burstId = burstIdLookup[forwardCount][leftCount];
    if (burstId !== "") {
        var burstSignals = parseInt(document.getElementById(burstId).value);
        command.push({
            "frequency": frequency,
            "dead_frequency": deadFrequency,
            "burst_us": parseFloat(document.getElementById("signal-burst-us").value),
            "spacing_us": parseInt(document.getElementById("sync-spacing-us").value),
            "repeats": burstSignals
        });
    }
    return JSON.stringify(command);
}
function sendControlMessage(evt) {
    if (evt !== undefined) {
        evt.preventDefault();
    }
    if (socket !== null) {
        socket.send(formatControlMessage());
    }
}
function keyHandler(evt, down) {
    evt = evt || window.evt;
    //var key = evt.key || evt.which || evt.keyCode;
    var needUpdate = false;  // We'll use this to ignore key repeats

      if (evt.key === "ArrowUp" || evt.keyCode === 38) {  // Up
        needUpdate = (forward !== down);
        forward = down;
    } else if (evt.key === "ArrowDown" || evt.keyCode === 40) {  // Down
        needUpdate = (reverse !== down);
        reverse = down;
    } else if (evt.key === "ArrowLeft" || evt.keyCode === 37) {  // Left
        needUpdate = (left !== down);
        left = down;
    } else if (evt.key === "ArrowRight" || evt.keyCode === 39) {  // Right
        needUpdate = (right !== down);
        right = down;
    }
    if (needUpdate) {
        sendControlMessage();
    }
}

function init(){
document.getElementById("control-file").addEventListener("change", handleFileSelect);
document.getElementById("connect").addEventListener("click", connect);
var body = document.getElementById("body");
body.addEventListener("keyup", function (evt) { keyHandler(evt, false); });
body.addEventListener("keydown", function (evt) { keyHandler(evt, true); });
}