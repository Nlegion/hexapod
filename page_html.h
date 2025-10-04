const char PROGMEM PAGE_HTML[] = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Hexapod Robot Control</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 10px;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: rgba(255, 255, 255, 0.95);
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            overflow: hidden;
        }
        
        .header {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 20px;
            text-align: center;
            color: white;
        }
        
        .header h1 { font-size: 2em; margin-bottom: 10px; }
        
        .status-bar {
            display: flex;
            justify-content: space-around;
            padding: 15px;
            background: #f8f9fa;
            border-bottom: 2px solid #dee2e6;
            flex-wrap: wrap;
            gap: 10px;
        }
        
        .status-item {
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        
        #connection-status { font-weight: bold; color: #dc3545; }
        #connection-status.connected { color: #28a745; }
        #command-status { font-weight: bold; color: #6c757d; }
        #command-status.executing { color: #ffc107; }
        #command-status.completed { color: #28a745; }
        
        .battery-container {
            width: 100px;
            height: 30px;
            border: 2px solid #333;
            border-radius: 5px;
            position: relative;
            background: #fff;
        }
        
        .battery-container::after {
            content: '';
            position: absolute;
            right: -6px;
            top: 50%;
            transform: translateY(-50%);
            width: 4px;
            height: 12px;
            background: #333;
            border-radius: 0 2px 2px 0;
        }
        
        .battery-level {
            height: 100%;
            transition: width 0.3s ease;
            border-radius: 3px;
        }
        
        .battery-high { background: #28a745; }
        .battery-medium { background: #ffc107; }
        .battery-low { background: #fd7e14; }
        .battery-critical { background: #dc3545; }
        
        #battery-text {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            font-size: 12px;
            font-weight: bold;
            color: #333;
            z-index: 10;
        }
        
        .tabs {
            display: flex;
            background: #e9ecef;
            border-bottom: 2px solid #dee2e6;
        }
        
        .tab-button {
            flex: 1;
            padding: 15px 20px;
            background: transparent;
            border: none;
            cursor: pointer;
            font-size: 16px;
            font-weight: bold;
            color: #6c757d;
            transition: all 0.3s;
        }
        
        .tab-button.active {
            background: white;
            color: #667eea;
            border-bottom: 3px solid #667eea;
        }
        
        .tab-button:hover { background: rgba(255,255,255,0.5); }
        
        .tab-content {
            display: none;
            padding: 20px;
            animation: fadeIn 0.3s;
        }
        
        .tab-content.active { display: block; }
        
        @keyframes fadeIn {
            from { opacity: 0; }
            to { opacity: 1; }
        }
        
        .control-section {
            margin-bottom: 30px;
            padding: 20px;
            background: white;
            border-radius: 15px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        
        .control-section h2, .control-section h3 {
            margin-bottom: 15px;
            color: #333;
            text-align: center;
        }
        
        .control-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            max-width: 400px;
            margin: 0 auto;
        }
        
        .test-grid {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 10px;
        }
        
        button {
            padding: 15px 20px;
            font-size: 16px;
            font-weight: bold;
            border: none;
            border-radius: 10px;
            cursor: pointer;
            transition: all 0.3s ease;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        
        button:active {
            transform: translateY(2px);
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        
        .movement {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }
        
        .movement:hover {
            background: linear-gradient(135deg, #764ba2 0%, #667eea 100%);
        }
        
        .stop { background: #dc3545; color: white; }
        .stop:hover { background: #c82333; }
        .test { background: #17a2b8; color: white; }
        .test:hover { background: #138496; }
        .diagnostic { background: #ffc107; color: #212529; }
        .diagnostic:hover { background: #e0a800; }
        
        .speed-toggle-container {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 15px;
            margin: 20px 0;
        }
        
        .speed-label { font-size: 18px; font-weight: bold; }
        
        .speed-switch {
            position: relative;
            display: inline-block;
            width: 60px;
            height: 34px;
        }
        
        .speed-switch input { opacity: 0; width: 0; height: 0; }
        
        .speed-slider {
            position: absolute;
            cursor: pointer;
            top: 0; left: 0; right: 0; bottom: 0;
            background-color: #6c757d;
            transition: 0.4s;
            border-radius: 34px;
        }
        
        .speed-slider:before {
            position: absolute;
            content: "";
            height: 26px;
            width: 26px;
            left: 4px;
            bottom: 4px;
            background-color: white;
            transition: 0.4s;
            border-radius: 50%;
        }
        
        input:checked + .speed-slider { background-color: #ff6b35; }
        input:checked + .speed-slider:before { transform: translateX(26px); }
        
        .hexapod-layout {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
            margin: 20px 0;
        }
        
        .hex-row {
            display: flex;
            justify-content: center;
            align-items: center;
            gap: 20px;
        }
        
        .leg-test {
            width: 80px;
            height: 80px;
            border-radius: 10px;
            background: #28a745;
            color: white;
            font-size: 14px;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        
        .leg-test:hover { background: #218838; }
        
        .hex-body {
            width: 100px;
            height: 100px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border-radius: 15px;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        
        .hex-symbol { font-size: 36px; color: white; font-weight: bold; }
        .hex-center-top, .hex-center-bottom { width: 100px; height: 20px; }
        
        #coordinates-display {
            max-height: 400px;
            overflow-y: auto;
            background: #f8f9fa;
            padding: 15px;
            border-radius: 10px;
            font-family: monospace;
        }
        
        .coord-info {
            text-align: center;
            color: #6c757d;
            padding: 20px;
        }
        
        .coord-phase h4 {
            color: #667eea;
            margin: 10px 0;
            padding: 5px;
            background: #e9ecef;
            border-radius: 5px;
        }
        
        .coord-leg {
            margin: 5px 0;
            padding: 8px;
            background: white;
            border-radius: 5px;
            border-left: 4px solid #667eea;
        }
        
        .coord-leg-name { font-weight: bold; margin-right: 10px; }
        .coord-left { color: #28a745; }
        .coord-right { color: #007bff; }
        
        @media (max-width: 768px) {
            body { padding: 5px; }
            .header h1 { font-size: 1.5em; }
            .status-bar { flex-direction: column; gap: 15px; }
            .control-section { padding: 15px; margin-bottom: 15px; }
            .control-grid { max-width: 100%; }
            .test-grid { grid-template-columns: repeat(2, 1fr); }
            button { padding: 12px 15px; font-size: 14px; }
            .leg-test { width: 60px; height: 60px; font-size: 12px; }
            .hex-body { width: 80px; height: 80px; }
            .hex-symbol { font-size: 24px; }
            .tab-button { padding: 12px 10px; font-size: 14px; }
        }
        
        @media (max-width: 480px) {
            .header h1 { font-size: 1.2em; }
            .test-grid { grid-template-columns: 1fr; }
            button { padding: 10px 12px; font-size: 13px; }
            .control-grid { gap: 8px; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🕷️ Hexapod Robot Control</h1>
            <p>ESP32-S3 WebSocket Interface</p>
        </div>

        <div class="status-bar">
            <div class="status-item">
                <div>Connection</div>
                <div id="connection-status">Connecting...</div>
            </div>
            <div class="status-item">
                <div>Command</div>
                <div id="command-status">Ready</div>
            </div>
            <div class="status-item">
                <div>Battery</div>
                <div class="battery-container">
                    <div id="battery-level" class="battery-level battery-high" style="width: 80%;"></div>
                    <div id="battery-text">12.0 V</div>
                </div>
            </div>
        </div>

        <div class="tabs">
            <button class="tab-button active" onclick="switchTab('actions')">🎮 Actions</button>
            <button class="tab-button" onclick="switchTab('tests')">🔧 Tests</button>
        </div>

        <div id="tab-actions" class="tab-content active">
            
            <div class="control-section">
                <h3>🏃 Gait Speed Control</h3>
                <div class="speed-toggle-container">
                    <span class="speed-label">🐌 Slow</span>
                    <label class="speed-switch">
                        <input type="checkbox" id="speedToggle" onchange="toggleSpeed()">
                        <span class="speed-slider"></span>
                    </label>
                    <span class="speed-label">🏃‍♂️ Fast</span>
                </div>
            </div>

            <div class="control-section">
                <h2>Movement Control</h2>
                <div class="control-grid">
                    <button></button>
                    <button class="movement" onclick="send('FWD')">↑</button>
                    <button></button>
                    <button class="movement" onclick="send('LEFT')">←</button>
                    <button class="stop" onclick="send('STOP')">STOP</button>
                    <button class="movement" onclick="send('RIGHT')">→</button>
                    <button></button>
                    <button class="movement" onclick="send('BWD')">↓</button>
                    <button></button>
                </div>
            </div>

            <div class="control-section">
                <h2>🎭 Gestures & Tricks</h2>
                <div class="test-grid">
                    <button class="test" onclick="send('SHAKE')" style="background: #9c27b0;">🤝 Shake Hand</button>
                    <button class="test" onclick="send('WAVE')" style="background: #9c27b0;">👋 Wave</button>
                    <button></button>
                    <button></button>
                </div>
            </div>

            <div class="control-section">
                <h2>📐 Body Adjustments</h2>
                <div class="test-grid">
                    <button class="test" onclick="send('LEAN_RIGHT')" style="background: #ff5722;">⬆️ Higher</button>
                    <button class="test" onclick="send('HEAD_UP')" style="background: #ff5722;">🔼 Head Up</button>
                    <button class="test" onclick="send('TWIST_LEFT')" style="background: #ff5722;">↶ Twist L</button>
                    <button class="test" onclick="send('BODY_DOWN')" style="background: #ff5722;">⬅️ Lean L</button>
                </div>
                <div class="test-grid" style="margin-top: 10px;">
                    <button class="test" onclick="send('LEAN_LEFT')" style="background: #ff5722;">⬇️ Lower</button>
                    <button class="test" onclick="send('HEAD_DOWN')" style="background: #ff5722;">🔽 Head Down</button>
                    <button class="test" onclick="send('TWIST_RIGHT')" style="background: #ff5722;">↷ Twist R</button>
                    <button class="test" onclick="send('BODY_UP')" style="background: #ff5722;">➡️ Lean R</button>
                </div>
            </div>

            <div class="control-section">
                <div style="text-align: center;">
                    <button class="stop" onclick="send('EMERGENCY')" style="font-size: 18px; padding: 15px 30px;">
                        ⚠️ EMERGENCY STOP ⚠️
                    </button>
                </div>
            </div>

        </div>

        <div id="tab-tests" class="tab-content">

            <div class="control-section">
                <h2>Diagnostic Tests</h2>
                <div class="test-grid">
                    <button class="diagnostic" onclick="send('DIAGNOSTIC')">Full Test</button>
                    <button class="diagnostic" onclick="send('JOINT_TEST')">Joint Directions</button>
                    <button class="diagnostic" onclick="send('RESET')">Reset All</button>
                    <button class="test" onclick="send('TRIPOD_TEST')">Tripod Test</button>
                </div>
            </div>

            <div class="control-section">
                <h2>Individual Leg Tests</h2>
                <div class="hexapod-layout">
                    <div class="hex-row">
                        <button class="leg-test front-left" onclick="send('TEST_LEG_5')">
                            FL<br>(5)
                        </button>
                        <div class="hex-center-top"></div>
                        <button class="leg-test front-right" onclick="send('TEST_LEG_0')">
                            FR<br>(0)
                        </button>
                    </div>
                    <div class="hex-row">
                        <button class="leg-test middle-left" onclick="send('TEST_LEG_4')">
                            ML<br>(4)
                        </button>
                        <div class="hex-body">
                            <div class="hex-symbol">X</div>
                        </div>
                        <button class="leg-test middle-right" onclick="send('TEST_LEG_1')">
                            MR<br>(1)
                        </button>
                    </div>
                    <div class="hex-row">
                        <button class="leg-test rear-left" onclick="send('TEST_LEG_3')">
                            RL<br>(3)
                        </button>
                        <div class="hex-center-bottom"></div>
                        <button class="leg-test rear-right" onclick="send('TEST_LEG_2')">
                            RR<br>(2)
                        </button>
                    </div>
                </div>
            </div>

            <div class="control-section">
                <h2>Live Coordinates</h2>
                <div id="coordinates-display">
                    <div class="coord-info">Execute TRIPOD_TEST to see live leg coordinates</div>
                </div>
            </div>

        </div>

    </div>

    <script>
        const connectionStatus = document.getElementById('connection-status');
        const commandStatus = document.getElementById('command-status');
        const coordinatesDisplay = document.getElementById('coordinates-display');
        let ws = null;

        function switchTab(tabName) {
            document.querySelectorAll('.tab-content').forEach(tab => {
                tab.classList.remove('active');
            });
            document.querySelectorAll('.tab-button').forEach(btn => {
                btn.classList.remove('active');
            });
            document.getElementById('tab-' + tabName).classList.add('active');
            event.target.classList.add('active');
        }

        function connect() {
            ws = new WebSocket(`ws://${location.hostname}:81/`);
            
            ws.onopen = function() {
                connectionStatus.textContent = 'Connected ✓';
                connectionStatus.className = 'connected';
                console.log("WebSocket connected");
            };
            
            ws.onclose = function() {
                connectionStatus.textContent = 'Disconnected';
                connectionStatus.className = '';
                setTimeout(connect, 2000);
            };
            
            ws.onerror = function() {
                connectionStatus.textContent = 'Connection Error';
                connectionStatus.className = '';
            };
            
            ws.onmessage = function(event) {
                console.log("Received:", event.data);
                handleWebSocketMessage(event.data);
            };
        }
        
        function handleWebSocketMessage(message) {
            if (message.startsWith('COORD:')) {
                displayCoordinates(message.substring(6));
            } else if (message.startsWith('PHASE:')) {
                displayPhase(message.substring(6));
            } else if (message.startsWith('CLEAR_COORDS')) {
                clearCoordinatesDisplay();
            } else if (message.startsWith('BATTERY:')) {
                const parts = message.split(':');
                if (parts.length >= 2) {
                    const voltage = parseFloat(parts[1]);
                    updateBatteryIndicator(voltage);
                }
            }
        }
        
        function updateBatteryIndicator(voltage) {
            const batteryLevel = document.getElementById('battery-level');
            const batteryText = document.getElementById('battery-text');
            
            batteryText.textContent = voltage.toFixed(2) + ' V';
            
            const voltageMax = 12.6;
            const voltageMin = 9.0;
            
            let percentage = ((voltage - voltageMin) / (voltageMax - voltageMin)) * 100;
            percentage = Math.max(0, Math.min(100, percentage));
            
            batteryLevel.style.width = percentage + '%';
            
            batteryLevel.className = 'battery-level';
            if (percentage > 60) {
                batteryLevel.classList.add('battery-high');
            } else if (percentage > 30) {
                batteryLevel.classList.add('battery-medium');
            } else if (percentage > 15) {
                batteryLevel.classList.add('battery-low');
            } else {
                batteryLevel.classList.add('battery-critical');
            }
        }
        
        function clearCoordinatesDisplay() {
            coordinatesDisplay.innerHTML = '<div class="coord-info">Execute TRIPOD_TEST to see live leg coordinates</div>';
        }
        
        function displayPhase(phaseInfo) {
            const phaseDiv = document.createElement('div');
            phaseDiv.className = 'coord-phase';
            phaseDiv.innerHTML = `<h4>${phaseInfo}</h4>`;
            
            if (phaseInfo.includes('PHASE 1') || phaseInfo.includes('ML (MIDDLE LEFT) DIAGNOSTIC')) {
                coordinatesDisplay.innerHTML = '';
            }
            
            coordinatesDisplay.appendChild(phaseDiv);
            coordinatesDisplay.scrollTop = coordinatesDisplay.scrollHeight;
        }
        
        function displayCoordinates(coordData) {
            const parts = coordData.split(':');
            if (parts.length >= 5) {
                const legName = parts[0];
                const type = parts[1];
                const coxa = parts[2];
                const femur = parts[3];
                const tibia = parts[4];
                
                const legDiv = document.createElement('div');
                legDiv.className = 'coord-leg';
                
                const isLeft = legName.includes('L');
                const colorClass = isLeft ? 'coord-left' : 'coord-right';
                
                legDiv.innerHTML = `
                    <span class="coord-leg-name ${colorClass}">${legName}:</span>
                    <span class="coord-values">COXA=${coxa}, FEMUR=${femur}, TIBIA=${tibia} [${type}]</span>
                `;
                
                coordinatesDisplay.appendChild(legDiv);
                coordinatesDisplay.scrollTop = coordinatesDisplay.scrollHeight;
            }
        }

        window.send = function(cmd) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(cmd);
                
                commandStatus.textContent = `Executing: ${cmd}`;
                commandStatus.className = 'executing';
                console.log("Sent command:", cmd);
                
                setTimeout(function() {
                    commandStatus.textContent = `Completed: ${cmd}`;
                    commandStatus.className = 'completed';
                }, 3000);
                
                setTimeout(function() {
                    commandStatus.textContent = 'Ready';
                    commandStatus.className = '';
                }, 5000);
                
            } else {
                console.log("Error: Not connected to send:", cmd);
                connectionStatus.textContent = 'Not Connected';
                connectionStatus.className = '';
                commandStatus.textContent = 'Connection Error';
                commandStatus.className = '';
            }
        }

        function toggleSpeed() {
            const toggle = document.getElementById('speedToggle');
            
            if (toggle.checked) {
                send('FAST');
                console.log("Speed switched to FAST");
            } else {
                send('SLOW');
                console.log("Speed switched to SLOW");
            }
        }

        connect();
    </script>
</body>
</html>
)=====";

