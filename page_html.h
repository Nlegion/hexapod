// HTML страница для управления гексаподом с диагностическими функциями
const char PROGMEM PAGE_HTML[] = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Hexapod Control & Debug</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 { text-align: center; color: #333; }
        h2 { color: #666; margin-top: 30px; }
        .control-section {
            margin: 20px 0;
            padding: 15px;
            border: 1px solid #ddd;
            border-radius: 8px;
        }
        .control-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            max-width: 200px;
            margin: 0 auto;
        }
        .test-grid {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 10px;
            margin: 10px 0;
        }
        
        /* Hexapod Layout */
        .hexapod-layout {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
            margin: 20px 0;
            padding: 20px;
            background: #f8f9fa;
            border-radius: 10px;
            border: 2px dashed #dee2e6;
        }
        
        .hex-row {
            display: flex;
            align-items: center;
            gap: 20px;
        }
        
        .hex-body {
            width: 60px;
            height: 60px;
            background: #e9ecef;
            border-radius: 50%;
            display: flex;
            align-items: center;
            justify-content: center;
            border: 2px solid #6c757d;
        }
        
        .hex-symbol {
            font-size: 24px;
            font-weight: bold;
            color: #495057;
        }
        
        .hex-center-top, .hex-center-bottom {
            width: 30px;
            height: 20px;
            background: #dee2e6;
            border-radius: 5px;
        }
        
        .leg-test {
            width: 70px;
            height: 50px;
            font-size: 12px;
            font-weight: bold;
            line-height: 1.2;
            border-radius: 8px;
            border: 2px solid #007bff;
        }
        
        .leg-test:hover {
            border-color: #0056b3;
            transform: scale(1.05);
        }
        
        .front-left, .middle-left, .rear-left {
            background: linear-gradient(135deg, #ff6b6b, #ee5a52);
            color: white;
        }
        
        .front-right, .middle-right, .rear-right {
            background: linear-gradient(135deg, #4ecdc4, #44a08d);
            color: white;
        }
        
        /* Header with status */
        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 20px;
        }
        
        .status-compact {
            display: flex;
            flex-direction: column;
            gap: 5px;
            min-width: 200px;
        }
        
        .status-compact > div {
            padding: 8px 12px;
            border-radius: 5px;
            font-size: 12px;
            font-weight: bold;
            text-align: center;
        }
        
        #connection-status {
            background: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }
        
        #connection-status.connected {
            background: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        
        #command-status {
            background: #cce7ff;
            color: #004085;
            border: 1px solid #b3d7ff;
            font-size: 11px;
        }
        
        #command-status.executing {
            background: #fff3cd;
            color: #856404;
            border: 1px solid #ffeaa7;
        }
        
        #command-status.completed {
            background: #d1ecf1;
            color: #0c5460;
            border: 1px solid #bee5eb;
        }
        
        /* Battery indicator */
        #battery-status {
            background: #fff;
            border: 2px solid #dee2e6;
            border-radius: 8px;
            padding: 8px 12px;
            font-size: 12px;
            font-weight: bold;
            display: flex;
            align-items: center;
            gap: 8px;
        }
        
        .battery-icon {
            width: 30px;
            height: 16px;
            border: 2px solid #333;
            border-radius: 3px;
            position: relative;
            display: inline-block;
        }
        
        .battery-icon::after {
            content: '';
            position: absolute;
            right: -4px;
            top: 4px;
            width: 3px;
            height: 6px;
            background: #333;
            border-radius: 0 2px 2px 0;
        }
        
        .battery-level {
            position: absolute;
            left: 2px;
            top: 2px;
            bottom: 2px;
            width: calc(100% - 4px);
            border-radius: 1px;
            transition: all 0.3s ease;
        }
        
        .battery-high { background: linear-gradient(90deg, #28a745, #20c997); }
        .battery-medium { background: linear-gradient(90deg, #ffc107, #fd7e14); }
        .battery-low { background: linear-gradient(90deg, #dc3545, #c82333); }
        .battery-critical { background: #dc3545; animation: blink 1s infinite; }
        
        @keyframes blink {
            0%, 50% { opacity: 1; }
            51%, 100% { opacity: 0.3; }
        }
        
         /* Coordinates display */
         #coordinates-display {
             background: #f8f9fa;
             border: 1px solid #dee2e6;
             border-radius: 5px;
             padding: 15px;
             font-family: 'Courier New', monospace;
             font-size: 12px;
             max-height: 400px;
             overflow-y: auto;
         }
         
         /* Стильный переключатель скорости */
         .speed-toggle-container {
             display: flex;
             align-items: center;
             justify-content: center;
             gap: 15px;
             margin: 20px 0;
         }
         
         .speed-label {
             font-size: 18px;
             font-weight: bold;
             color: #495057;
         }
         
         .speed-switch {
             position: relative;
             display: inline-block;
             width: 80px;
             height: 40px;
         }
         
         .speed-switch input {
             opacity: 0;
             width: 0;
             height: 0;
         }
         
         .speed-slider {
             position: absolute;
             cursor: pointer;
             top: 0;
             left: 0;
             right: 0;
             bottom: 0;
             background-color: #6c757d;
             border-radius: 20px;
             transition: .4s;
             box-shadow: 0 2px 4px rgba(0,0,0,0.2);
         }
         
         .speed-slider:before {
             position: absolute;
             content: "";
             height: 32px;
             width: 32px;
             left: 4px;
             bottom: 4px;
             background-color: white;
             border-radius: 50%;
             transition: .4s;
             box-shadow: 0 2px 4px rgba(0,0,0,0.3);
         }
         
         .speed-switch input:checked + .speed-slider {
             background-color: #ff6b35;
         }
         
         .speed-switch input:checked + .speed-slider:before {
             transform: translateX(40px);
         }
         
         .speed-slider:hover {
             box-shadow: 0 4px 8px rgba(0,0,0,0.3);
         }
         
         #speed-indicator {
             padding: 8px 16px;
             border-radius: 20px;
             background: #e9ecef;
             display: inline-block;
             min-width: 120px;
             transition: all 0.3s ease;
         }
        
        .coord-info {
            color: #6c757d;
            font-style: italic;
            text-align: center;
            margin: 20px 0;
        }
        
        .coord-phase {
            background: #e9ecef;
            border-left: 4px solid #007bff;
            padding: 10px;
            margin: 10px 0;
            border-radius: 0 5px 5px 0;
        }
        
        .coord-phase h4 {
            margin: 0 0 8px 0;
            color: #007bff;
            font-size: 14px;
        }
        
        .coord-leg {
            display: flex;
            justify-content: space-between;
            padding: 3px 0;
            border-bottom: 1px solid #dee2e6;
        }
        
        .coord-leg:last-child {
            border-bottom: none;
        }
        
        .coord-leg-name {
            font-weight: bold;
            min-width: 100px;
        }
        
        .coord-values {
            flex-grow: 1;
            text-align: right;
        }
        
        .coord-left {
            color: #dc3545;
        }
        
        .coord-right {
            color: #28a745;
        }
        
        button {
            padding: 12px;
            font-size: 14px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            transition: all 0.3s;
        }
        .movement { background-color: #28a745; color: white; }
        .movement:hover { background-color: #218838; }
        .stop { background-color: #dc3545; color: white; }
        .stop:hover { background-color: #c82333; }
        .test { background-color: #ffc107; color: #212529; }
        .test:hover { background-color: #e0a800; }
        .diagnostic { background-color: #17a2b8; color: white; }
        .diagnostic:hover { background-color: #138496; }
        .leg-test { background-color: #fd7e14; color: white; }
        .leg-test:hover { background-color: #e55f00; }
        #status {
            margin-top: 20px;
            padding: 10px;
            background: #e9ecef;
            border-radius: 5px;
            font-family: monospace;
            white-space: pre-wrap;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🕷️ Hexapod Robot Control</h1>
            <div class="status-compact">
                <div id="connection-status">Connecting...</div>
                <div id="battery-status">
                    <div class="battery-icon">
                        <div id="battery-level" class="battery-level battery-high" style="width: 100%;"></div>
                    </div>
                    <span id="battery-text">-- V</span>
                </div>
                <div id="command-status">Ready</div>
            </div>
        </div>
        
        <!-- Gait Speed Control - теперь ДО Movement Control -->
        <div class="control-section">
             <h3>🏃 Gait Speed Control</h3>
             
             <!-- Toggle Switch для Fast/Slow -->
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
                <button class="test" onclick="send('LEAN_LEFT')" style="background: #ff5722;"> ⬇️ Lower</button>
                <button class="test" onclick="send('HEAD_DOWN')" style="background: #ff5722;">🔽 Head Down</button>
                <button class="test" onclick="send('TWIST_RIGHT')" style="background: #ff5722;">↷ Twist R</button>
                <button class="test" onclick="send('BODY_UP')" style="background: #ff5722;">➡️ Lean R</button>
            </div>
        </div>

        <div class="control-section">
            <h2>Diagnostic Tests</h2>
            <div class="test-grid">
                <button class="diagnostic" onclick="send('DIAGNOSTIC')">Full Test</button>
                <button class="diagnostic" onclick="send('JOINT_TEST')">Joint Directions</button>
                <button class="diagnostic" onclick="send('RESET')">Reset All</button>
                <button class="test" onclick="send('TRIPOD_TEST')">Tripod Test</button>
            </div>
            <div style="margin-top: 15px; text-align: center;">
                <button class="stop" onclick="send('EMERGENCY')" style="font-size: 18px; padding: 15px 30px;">
                    ⚠️ EMERGENCY STOP ⚠️
                </button>
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

    <script>
        const connectionStatus = document.getElementById('connection-status');
        const commandStatus = document.getElementById('command-status');
        const coordinatesDisplay = document.getElementById('coordinates-display');
        let ws = null;

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
                // Координаты от TRIPOD_TEST
                displayCoordinates(message.substring(6)); // убираем "COORD:"
            } else if (message.startsWith('PHASE:')) {
                // Начало новой фазы
                displayPhase(message.substring(6)); // убираем "PHASE:"
            } else if (message.startsWith('CLEAR_COORDS')) {
                // Очищаем дисплей координат
                clearCoordinatesDisplay();
            } else if (message.startsWith('BATTERY:')) {
                // Обновляем индикатор батареи
                const voltage = parseFloat(message.substring(8)); // убираем "BATTERY:"
                updateBatteryIndicator(voltage);
            }
        }
        
        function updateBatteryIndicator(voltage) {
            const batteryLevel = document.getElementById('battery-level');
            const batteryText = document.getElementById('battery-text');
            
            // Обновляем текст
            batteryText.textContent = voltage.toFixed(2) + ' V';
            
            // Настройка для Li-Ion/Li-Po батареи (типично 3S = 9.0-12.6V)
            // Можно настроить под вашу конфигурацию батареи
            const voltageMax = 12.6; // Полностью заряжена (3S Li-Po)
            const voltageMin = 9.0;   // Разряжена (критический уровень)
            
            // Вычисляем процент
            let percentage = ((voltage - voltageMin) / (voltageMax - voltageMin)) * 100;
            percentage = Math.max(0, Math.min(100, percentage)); // Ограничиваем 0-100%
            
            // Обновляем ширину индикатора
            batteryLevel.style.width = percentage + '%';
            
            // Меняем цвет в зависимости от уровня
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
            
            console.log(`Battery updated: ${voltage}V (${percentage.toFixed(1)}%)`);
        }
        
        function clearCoordinatesDisplay() {
            coordinatesDisplay.innerHTML = '<div class="coord-info">Execute TRIPOD_TEST to see live leg coordinates</div>';
        }
        
        function displayPhase(phaseInfo) {
            // Добавляем заголовок фазы
            const phaseDiv = document.createElement('div');
            phaseDiv.className = 'coord-phase';
            phaseDiv.innerHTML = `<h4>${phaseInfo}</h4>`;
            
            // Очищаем предыдущие координаты если это первая фаза
            if (phaseInfo.includes('PHASE 1') || phaseInfo.includes('ML (MIDDLE LEFT) DIAGNOSTIC')) {
                coordinatesDisplay.innerHTML = '';
            }
            
            coordinatesDisplay.appendChild(phaseDiv);
            coordinatesDisplay.scrollTop = coordinatesDisplay.scrollHeight;
        }
        
        function displayCoordinates(coordData) {
            // Парсим данные координат: "LEG_NAME:TYPE:COXA:FEMUR:TIBIA"
            const parts = coordData.split(':');
            if (parts.length >= 5) {
                const legName = parts[0];
                const type = parts[1]; // LIFTING, GROUND, etc.
                const coxa = parts[2];
                const femur = parts[3];
                const tibia = parts[4];
                
                const legDiv = document.createElement('div');
                legDiv.className = 'coord-leg';
                
                // Определяем цвет (левая или правая нога)
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
                 
                 // Показываем статус команды
                 commandStatus.textContent = `Executing: ${cmd}`;
                 commandStatus.className = 'executing';
                 console.log("Sent command:", cmd);
                 
                 // Через 3 секунды показываем "завершено"
                 setTimeout(function() {
                     commandStatus.textContent = `Completed: ${cmd}`;
                     commandStatus.className = 'completed';
                 }, 3000);
                 
                 // Через 5 секунд возвращаем к "Ready"
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

         // Функция переключения скорости Fast/Slow
         function toggleSpeed() {
             const toggle = document.getElementById('speedToggle');
             
             if (toggle.checked) {
                 // Включен = Fast
                 send('FAST');
                 console.log("Speed switched to FAST");
             } else {
                 // Выключен = Slow  
                 send('SLOW');
                 console.log("Speed switched to SLOW");
             }
         }

        connect();
    </script>
</body>
</html>
)=====";
