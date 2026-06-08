// Check if a robot was previously saved in the browser, otherwise default to 1
let activeRobot = localStorage.getItem("lastRobot") ? parseInt(localStorage.getItem("lastRobot")) : 1;

// When the page loads, make sure the correct radio button is physically checked
window.addEventListener('DOMContentLoaded', (event) => {
  const radioBtn = document.querySelector(`input[name="robotSel"][value="${activeRobot}"]`);
  if (radioBtn) radioBtn.checked = true;
});

function switchRobot(id) {
  activeRobot = id;
  localStorage.setItem("lastRobot", id); // Save the choice to the browser memory
  isSynced = false; // Force the sliders to snap to the new robot's EEPROM values
}

let isSynced = false;
const gateway = `ws://${window.location.hostname}:81/`;
let websocket;
let logData = [
  [
    "Time",
    "Angle",
    "Target",
    "Error",
    "P",
    "I",
    "D",
    "SpeedIn",
    "PosErr",
    "MotL",
    "MotR",
    "Battery",
  ],
];
let isRec = false;
let startTime = Date.now();
let sampleDiv = 1;
let packetCount = 0;

const MAX_PTS = 150;
const d = {
  ang: [],
  targ: [],
  err: [],
  p: [],
  i: [],
  d: [],
  spd: [],
  mL: [],
  mR: [],
};

function initWebSocket() {
  websocket = new WebSocket(gateway);
  websocket.onmessage = onMessage;
}

let cmdQueue = [];
let isSendingCmd = false;

function sendCmd(prefix, val) {
  // Push the command to the waiting line instead of sending it instantly
  cmdQueue.push(activeRobot + "|" + prefix + ":" + val);
  processCmdQueue();
}

function processCmdQueue() {
  // If we are currently sending, or the line is empty, do nothing
  if (isSendingCmd || cmdQueue.length === 0) return;
  
  if (websocket.readyState == 1) {
    isSendingCmd = true;
    websocket.send(cmdQueue.shift()); // Send the first item in the line
    
    // Wait exactly 40ms to let the ESP32 ESP-NOW radio finish transmitting
    setTimeout(() => {
      isSendingCmd = false;
      processCmdQueue(); // Check if there is anything else waiting in line
    }, 40); 
  }
}

function updateRate() {
  sampleDiv = parseInt(document.getElementById("sRate").value);
}

function toggleRec() {
  isRec = !isRec;
  document.getElementById("recBtn").innerText = isRec
    ? "Recording..."
    : "Start Recording";
  if (isRec) startTime = Date.now();
}

function exportCSV() {
  let csvContent =
    "data:text/csv;charset=utf-8," + logData.map((e) => e.join(",")).join("\n");
  const encodedUri = encodeURI(csvContent);
  const link = document.createElement("a");
  link.setAttribute("href", encodedUri);
  link.setAttribute("download", "telemetry.csv");
  document.body.appendChild(link);
  link.click();
}

function onMessage(event) {
  const raw = event.data.split(",");

// We now expect 18 variables
  if (raw.length == 18) {
    const vals = raw.map(Number);
    const telemetryRobotId = vals[0];

    if (telemetryRobotId !== activeRobot) return; 

    const tNow = ((Date.now() - startTime) / 1000).toFixed(2);

    if (!isSynced) {
      document.getElementById("P").value = vals[12];
      document.getElementById("P_v").innerText = vals[12];
      document.getElementById("D").value = vals[13];
      document.getElementById("D_v").innerText = vals[13];
      document.getElementById("V").value = vals[14];
      document.getElementById("V_v").innerText = vals[14];
      document.getElementById("U").value = vals[15];
      document.getElementById("U_v").innerText = vals[15];
      document.getElementById("Y").value = vals[16];
      document.getElementById("Y_v").innerText = vals[16];
      
      // Add the new Trim sync here!
      document.getElementById("O").value = vals[17];
      document.getElementById("O_v").innerText = vals[17];
      
      isSynced = true;
    }

    document.getElementById("bat-read").innerText = vals[11].toFixed(2) + " V";
    document.getElementById("bat-read").style.color =
      vals[11] < 10.5 ? "red" : "green";

    // Only log the first 11 variables to keep your CSV clean
    if (isRec) logData.push([tNow].concat(vals.slice(0, 11)));

    packetCount++;
    if (packetCount % sampleDiv === 0) {
      d.ang.push(vals[0]);
      d.targ.push(vals[1]);
      d.err.push(vals[2]);
      d.p.push(vals[3]);
      d.i.push(vals[4]);
      d.d.push(vals[5]);
      d.spd.push(vals[6]);
      d.mL.push(vals[8]);
      d.mR.push(vals[9]);

      for (const key in d) {
        if (d[key].length > MAX_PTS) d[key].shift();
      }
      drawGraphs();
    }
  }
}

function drawLine(ctx, arr, color, height, scaleMultiplier) {
  ctx.beginPath();
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  const step = ctx.canvas.width / MAX_PTS;
  for (let i = 0; i < arr.length; i++) {
    const x = i * step;
    const y = height / 2 - arr[i] * scaleMultiplier;
    if (i == 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function drawGraphs() {
  const cTop = document.getElementById("graphTop");
  const ctxT = cTop.getContext("2d");
  const cBot = document.getElementById("graphBot");
  const ctxB = cBot.getContext("2d");
  cTop.width = cTop.clientWidth;
  cTop.height = cTop.clientHeight;
  cBot.width = cBot.clientWidth;
  cBot.height = cBot.clientHeight;

  ctxT.clearRect(0, 0, cTop.width, cTop.height);
  ctxB.clearRect(0, 0, cBot.width, cBot.height);

  ctxT.beginPath();
  ctxT.moveTo(0, cTop.height / 2);
  ctxT.lineTo(cTop.width, cTop.height / 2);
  ctxT.strokeStyle = "#ddd";
  ctxT.stroke();
  ctxB.beginPath();
  ctxB.moveTo(0, cBot.height / 2);
  ctxB.lineTo(cBot.width, cBot.height / 2);
  ctxB.strokeStyle = "#ddd";
  ctxB.stroke();

  // ---- DYNAMIC AUTO-SCALING ALGORITHM ----
  let maxTop = 5;
  for (let i = 0; i < d.ang.length; i++) {
    maxTop = Math.max(
      maxTop,
      Math.abs(d.ang[i]),
      Math.abs(d.targ[i]),
      Math.abs(d.err[i]),
    );
  }
  const scaleTop = cTop.height / 2 / (maxTop * 1.1);

  let maxBot = 10;
  for (let i = 0; i < d.p.length; i++) {
    maxBot = Math.max(
      maxBot,
      Math.abs(d.p[i]),
      Math.abs(d.i[i]),
      Math.abs(d.d[i]),
      Math.abs(d.spd[i]),
      Math.abs(d.mL[i]),
      Math.abs(d.mR[i]),
    );
  }
  const scaleBot = cBot.height / 2 / (maxBot * 1.1);

  // --- UPGRADED GRID AND NUMBERS ---
  function drawGrid(ctx, height, width, max, scale) {
    ctx.fillStyle = "#333";
    ctx.font = "12px sans-serif";
    ctx.strokeStyle = "#e0e0e0"; // Light gray color for the grid lines
    ctx.lineWidth = 1;

    // Draw lines at Max, Half-Max, Zero, Half-Min, and Min
    const steps = [1, 0.5, 0, -0.5, -1];
    
    for (let i = 0; i < steps.length; i++) {
      const s = steps[i];
      // Calculate the physical Y pixel coordinate on the canvas
      const y = (height / 2) - (max * 1.1 * s * scale);
      
      // Draw the horizontal grid line
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();

      // Draw the number label just above the line
      const valText = (max * 1.1 * s).toFixed(1);
      
      // Prevent the very top label from drawing off the screen
      if (s === 1) {
        ctx.fillText(valText, 5, y + 12);
      } else {
        ctx.fillText(valText, 5, y - 4);
      }
    }
  }

  // Apply the grid to both the top and bottom graphs
  drawGrid(ctxT, cTop.height, cTop.width, maxTop, scaleTop);
  drawGrid(ctxB, cBot.height, cBot.width, maxBot, scaleBot);
  // ----------------------------------

  if (document.getElementById("cAng").checked)
    drawLine(ctxT, d.ang, "red", cTop.height, scaleTop);
  if (document.getElementById("cTarg").checked)
    drawLine(ctxT, d.targ, "blue", cTop.height, scaleTop);
  if (document.getElementById("cErr").checked)
    drawLine(ctxT, d.err, "purple", cTop.height, scaleTop);

  if (document.getElementById("cP").checked)
    drawLine(ctxB, d.p, "red", cBot.height, scaleBot);
  if (document.getElementById("cI").checked)
    drawLine(ctxB, d.i, "green", cBot.height, scaleBot);
  if (document.getElementById("cD").checked)
    drawLine(ctxB, d.d, "blue", cBot.height, scaleBot);
  if (document.getElementById("cSpd").checked)
    drawLine(ctxB, d.spd, "orange", cBot.height, scaleBot);
  if (document.getElementById("cMotL").checked)
    drawLine(ctxB, d.mL, "black", cBot.height, scaleBot);
  if (document.getElementById("cMotR").checked)
    drawLine(ctxB, d.mR, "gray", cBot.height, scaleBot);
}

const zone = document.getElementById("joystick-zone");
const stick = document.getElementById("joystick-stick");
let active = false;
const maxDiff = 70;
let lastJoySend = 0;

zone.addEventListener("mousedown", startDrag);
zone.addEventListener("touchstart", startDrag, { passive: false });
document.addEventListener("mouseup", endDrag);
document.addEventListener("touchend", endDrag);
document.addEventListener("mousemove", drag);
document.addEventListener("touchmove", drag, { passive: false });

function startDrag(e) {
  active = true;
}

function endDrag(e) {
  if (!active) return;
  active = false;
  stick.style.transform = `translate(0px, 0px)`;

  // Send both commands instantly. The new Queue will automatically space them out!
  sendCmd("B", 0);
  sendCmd("T", 0);
}

function drag(e) {
  if (!active) return;
  e.preventDefault();
  const rect = zone.getBoundingClientRect();
  const clientX = e.touches ? e.touches[0].clientX : e.clientX;
  const clientY = e.touches ? e.touches[0].clientY : e.clientY;

  let x = clientX - rect.left - 100;
  let y = clientY - rect.top - 100;
  const distance = Math.sqrt(x * x + y * y);
  if (distance > maxDiff) {
    x = x * (maxDiff / distance);
    y = y * (maxDiff / distance);
  }

  stick.style.transform = `translate(${x}px, ${y}px)`;

  const now = Date.now();
  if (now - lastJoySend > 100) {
    const MAX_FWD = parseFloat(document.getElementById("joySpeed").value);
    const MAX_SPIN = 50.0;

    let fwd = (y / maxDiff) * MAX_FWD;
    // Added negative sign back to x to flip Left/Right steering
    let spin = -(x / maxDiff) * MAX_SPIN;

    if (fwd > 0.5) {
      spin = -spin;
    }

    sendCmd("B", fwd.toFixed(1));
    sendCmd("T", spin.toFixed(1));
    lastJoySend = now;
  }
}

// --- MANUAL BUTTON & SERVO LOGIC WITH THROTTLES ---
let lastBtnSend = 0;
let lastServoSend = 0;

let btnInterval = null;

function btnDrive(fwdDir, spinDir) {
  const MAX_FWD = parseFloat(document.getElementById("joySpeed").value);
  const MAX_SPIN = 50.0;

  let fwd = fwdDir * MAX_FWD;
  let spin = spinDir * MAX_SPIN;

  sendCmd("B", fwd.toFixed(1));
  sendCmd("T", spin.toFixed(1));
}

// Fires when you push the button down
function startBtnDrive(fwdDir, spinDir) {
  if (btnInterval) clearInterval(btnInterval); // Clear any old timers
  btnDrive(fwdDir, spinDir); // Send the first command instantly

  // Keep sending the command every 100ms while held down
  btnInterval = setInterval(() => {
    btnDrive(fwdDir, spinDir);
  }, 100);
}

// Fires when you let go of the button
function stopBtnDrive() {
  if (btnInterval) {
    clearInterval(btnInterval);
    btnInterval = null;
  }
  // Send the hard stop commands
  sendCmd("B", 0);
  sendCmd("T", 0);
}

function moveServo(angle) {
  const now = Date.now();

  // 100ms throttle prevents flooding if the user rapidly drums their fingers on the buttons
  if (now - lastServoSend > 100) {
    sendCmd("G", angle);
    lastServoSend = now;
  }
}

window.onload = initWebSocket;
