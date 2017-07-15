
var optionToId = {
    "frequency": "frequency",
    "synchronization_burst_us": "sync-burst-us",
    "synchronization_spacing_us": "sync-spacing-us",
    "total_synchronizations": "total-syncs",
    "signal_burst_us": "signal-burst-us",
    "signal_spacing_us": "signal-spacing-us"
};
var forward = false;
var reverse = false;
var left = false;
var right = false;

function loadFile(file) {
    var reader = new FileReader();
    reader.onload = function (e) {
        var json;
        try {
            json = JSON.parse(e.target.result);
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
    var leftCount = left ? 0 : right ? 2 : 1;
    var burstIdLookup = [
        ["reverse-left", "reverse", "reverse-right"],
        ["left", "", "right"],  // Not moving
        ["forward-left", "forward", "forward-right"]
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

    var commandEndPoint = window.location.origin + '/command/';

    var xhr = new XMLHttpRequest();
    xhr.open('POST', commandEndPoint, true);
    xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
    xhr.onreadystatechange = function() {
        if (xhr.readyState === XMLHttpRequest.DONE) {
            if (xhr.status === 200) {
                // All good
            } else {
                writeMessage('Bad response received, HTTP code: ' + xhr.status);
            }
        }
    };
    xhr.onerror = function(err) {
        writeMessage('There was an error communicating with the backend: ' + err);
    };
    xhr.send('data=' + formatControlMessage());
}
function keyHandler(evt, down) {
    evt = evt || window.evt;
    var key = evt.key || evt.srcElement || evt.keyCode || evt.which || evt.changedTouches[0].target;
    var needUpdate = false;  // We'll use this to ignore key repeats

    if (key === "ArrowUp" || key === 38 || key === document.getElementById('forward-button')) {  // Up
        needUpdate = (forward !== down);
        forward = down;
    } else if (key === "ArrowDown" || key === 40 || key === document.getElementById('reverse-button')) {  // Down
        needUpdate = (reverse !== down);
        reverse = down;
    } else if (key === "ArrowLeft" || key === 37 || key === document.getElementById('left-button')) {  // Left
        needUpdate = (left !== down);
        left = down;
    } else if (key === "ArrowRight" || key === 39 || key === document.getElementById('right-button')) {  // Right
        needUpdate = (right !== down);
        right = down;
    }
    if (needUpdate) {
        sendControlMessage();
        evt.preventDefault();
    }
}
function addToggleOptionsListener(headingId, optionsId) {
    document.getElementById(optionsId).style.display = 'none';
    document.getElementById(headingId).addEventListener('click', function() {
        var heading = document.getElementById(headingId);
        var text = heading.innerText;
        var options = document.getElementById(optionsId);
        if (options.style.display == 'none') {
            heading.innerText = text.substr(0, text.length - 1) + '▲';
            options.style.display = 'block';
        } else {
            heading.innerText = text.substr(0, text.length - 1) + '▼';
            options.style.display = 'none';
        }
    }.bind(this));
}

var clearMessageTimeoutId = null;
function writeMessage(message) {
    var messageElement = document.getElementById('message');
    messageElement.innerText = message;
    if (clearMessageTimeoutId !== null) {
        window.clearTimeout(clearMessageTimeoutId);
    }
    clearMessageTimeoutId = window.setTimeout(
            function() { writeMessage(''); },
            2000);
}

function init(){
addToggleOptionsListener('parameters-heading', 'parameters-options');

document.getElementById("parameter-file").addEventListener("change", handleFileSelect);

['forward-button', 'reverse-button', 'left-button', 'right-button'].forEach(function (id) {
    var element = document.getElementById(id);
    ['mousedown', 'touchstart'].forEach(function (action) {
        element.addEventListener(action, function (evt) { keyHandler(evt, true); });
    });
    ['mouseup', 'touchend'].forEach(function (action) {
        element.addEventListener(action, function (evt) { keyHandler(evt, false); });
    });
});
var body = document.getElementById('body');
body.addEventListener('keydown', function (evt) { keyHandler(evt, true); });
body.addEventListener('keyup', function (evt) { keyHandler(evt, false); });
}
