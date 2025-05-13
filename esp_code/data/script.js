function toggleCheckbox(element) {
    var xhr = new XMLHttpRequest();
    if(element.checked){
      xhr.open("GET", "/update?state=1", true);
      document.getElementById("state").innerHTML = "ON";
    }
    else {
      xhr.open("GET", "/update?state=0", true);
      document.getElementById("state").innerHTML = "OFF";
    }
    xhr.send();
}

function logoutButton() {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/logout", true);
    xhr.send();
    setTimeout(function(){ window.open("/logged-out","_self"); }, 1000);
}


function updateCompass(pos) {
	document.getElementById('compassValue').innerText = `${pos}°`;
}


function sendCompassValue(pos) {
	fetch(`/compass?value=${pos}`);
	console.log("Compass value", pos);
}


function forwards500() { move('forwards', 500); }

function forwards20() { move('forwards', 20); }

function move(dir, dis) {
	fetch(`/${dir}${dis}`);
	console.log("Drive", dir, dis);
}

const currentCompassValue = document.getElementById('currentCompassValue');
setInterval(fetchCompassData, 500);
async function fetchCompassData() {
  try {
    const response = await fetch('/compassData');
    const compassData = await response.text();
    const	compassDegree = parseFloat(compassData);

    function getDirection(degrees) {
      if (degrees >= 337.5 || degrees < 22.5) return "N";
      if (degrees >= 22.5 && degrees < 67.5) return "NE";
      if (degrees >= 67.5 && degrees < 112.5) return "E";
      if (degrees >= 112.5 && degrees < 157.5) return "SE";
      if (degrees >= 157.5 && degrees < 202.5) return "S";
      if (degrees >= 202.5 && degrees < 247.5) return "SW";
      if (degrees >= 247.5 && degrees < 292.5) return "W";
      if (degrees >= 292.5 && degrees < 337.5) return "NW";
      return ""; // fallback
    }

    const compassDirection = getDirection(compassDegree);

    if (currentCompassValue) currentCompassValue.innerText = `${compassDegree.toFixed(1)} ° ${compassDirection}`;
  } catch (error) {
    console.error('Error fetching compass data:', error);
  }
}

const lidarText = document.getElementById('lidarText');
setInterval(fetchLidarData, 500);
const warningDiv = document.getElementById('warning');
async function fetchLidarData() {
  try {
    const response = await fetch('/lidar');
    const data = await response.text();
    const lidarValue = parseInt(data, 10);

    if (lidarText) {
      lidarText.innerText = `${lidarValue} cm`;
    }

    if (lidarValue < 10) {
      warningDiv.innerText = 'Warning: too close!';
      warningDiv.style.color = 'red';
    } else {
      warningDiv.innerText = '';
    }
  } catch (error) {
    console.error('Error fetching LIDAR data:', error);
  }
}

const colorText = document.getElementById('detectedColor');
setInterval(fetchColorData, 500);
async function fetchColorData() {
  try {
    const response = await fetch('/color');
    const data = await response.text();
    console.log(data);
    const colors = data.split(" ").slice(1, 4);

    if (data)
      colorText.style.backgroundColor = `rgb(${colors[0]},${colors[1]},${colors[2]})`;
    } catch (error) {
    console.error('Error fetching LIDAR data:', error);
  }
}

const encoderText = document.getElementById('encoderText');
setInterval(fetchEncoderData, 500);
async function fetchEncoderData() {
  try {
    const response = await fetch('/encoder');
    const data = await response.text();
    const encoderValue = parseFloat(data);
    if (encoderText) encoderText.innerText = `${encoderValue.toFixed(2)} mm/pulse`;
  }
  catch (error) {
    console.error('Error fetching ENCODER data:', error);
  }
}
