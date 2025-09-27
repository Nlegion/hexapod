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
        <h1>🕷️ Hexapod Robot Control</h1>
        
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
            <h2>Diagnostic Tests</h2>
            <div class="test-grid">
                <button class="diagnostic" onclick="send('CALIBRATE')">Calibrate</button>
                <button class="diagnostic" onclick="send('DIAGNOSTIC')">Full Test</button>
                <button class="diagnostic" onclick="send('RESET')">Reset All</button>
                <button class="test" onclick="send('TRIPOD_TEST')">Tripod Test</button>
            </div>
        </div>

        <div class="control-section">
            <h2>Individual Leg Tests</h2>
            <div class="test-grid">
                <button class="leg-test" onclick="send('TEST_LEG_0')">FR (0)</button>
                <button class="leg-test" onclick="send('TEST_LEG_1')">MR (1)</button>
                <button class="leg-test" onclick="send('TEST_LEG_2')">RR (2)</button>
                <button class="leg-test" onclick="send('TEST_LEG_3')">RL (3)</button>
                <button class="leg-test" onclick="send('TEST_LEG_4')">ML (4)</button>
                <button class="leg-test" onclick="send('TEST_LEG_5')">FL (5)</button>
            </div>
        </div>

        <div class="control-section">
            <h2>Connection Status</h2>
            <div id="status">Connecting...</div>
        </div>
    </div>

    <script>
        const status = document.getElementById('status');
        let ws = null;

        function connect() {
            ws = new WebSocket(`ws://${location.hostname}:81/`);
            
            ws.onopen = function() {
                status.textContent = 'Connected to Hexapod ✓';
                status.style.background = '#d4edda';
            };
            
            ws.onclose = function() {
                status.textContent = 'Disconnected. Reconnecting...';
                status.style.background = '#f8d7da';
                setTimeout(connect, 2000);
            };
            
            ws.onerror = function() {
                status.textContent = 'Connection error';
                status.style.background = '#f8d7da';
            };
        }

        window.send = function(cmd) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(cmd);
                status.textContent += '\n> ' + cmd;
                console.log("Sent command:", cmd);
            } else {
                status.textContent += '\nError: Not connected';
            }
        }

        connect();
    </script>
</body>
</html>
)=====";
