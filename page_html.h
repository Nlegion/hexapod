// HTML страница (без изменений)
const char PROGMEM PAGE_HTML[] = R"=====(
<!-- page.html -->
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Hexapod Control</title>
    <style>
        body {
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
            background-color: #f0f0f0;
        }
        .control {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            grid-template-rows: repeat(3, 1fr);
            gap: 10px;
            width: 200px;
            height: 200px;
        }
        button {
            width: 60px;
            height: 60px;
            font-size: 24px;
            background-color: #007BFF;
            color: white;
            border: none;
            border-radius: 5px;
            cursor: pointer;
        }
        button:hover {
            background-color: #0056b3;
        }
    </style>
</head>
<body>
    <div class="control">
        <button></button>
        <button onclick="send('FWD')">↑</button>
        <button></button>
        <button onclick="send('LEFT')">←</button>
        <button onclick="send('STOP')">⏹</button>
        <button onclick="send('RIGHT')">→</button>
        <button></button>
        <button onclick="send('BWD')">↓</button>
        <button></button>
    </div>
    <script>
    const ws = new WebSocket(`ws://${location.hostname}:81/ws`);
    
    // Глобальная функция для кнопок
    window.send = function(cmd) {
        ws.send(cmd);
        console.log("Sent command:", cmd);
    }
</script> 
</body>
</html>



)=====";
