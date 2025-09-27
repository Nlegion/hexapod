// HTML страница (без изменений)
const char PROGMEM PAGE_HTML[] = R"=====(
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>ESP8266 Spider Hexapod</title>
    <style>
        body {
            background-color: #808080;
            font-family: Arial, Helvetica, sans-serif;
            color: #000000;
        }
        #JD {
            text-align: center;
            font-family: "Lucida Sans Unicode", "Lucida Grande", sans-serif;
            font-size: 24px;
        }
        .foot {
            text-align: center;
            font-family: "Comic Sans MS", cursive;
            font-size: 9px;
            color: #F00;
        }
        .button {
            border: none;
            color: white;
            padding: 20px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            margin: 4px 2px;
            cursor: pointer;
            border-radius: 12px;
            width: 100%;
        }
        .red { background-color: #F00; }
        .green { background-color: #090; }
        .yellow { background-color: #F90; }
        .blue { background-color: #03C; }
    </style>
    <script>
        var websock;

        function start() {
            try {
                websock = new WebSocket('ws://' + window.location.hostname + ':81/');
                websock.onopen = function(evt) {
                    console.log('WebSocket открыт');
                };
                websock.onclose = function(evt) {
                    console.log('WebSocket закрыт');
                };
                websock.onerror = function(evt) {
                    console.error('Ошибка WebSocket:', evt);
                };
                websock.onmessage = function(evt) {
                    console.log('Получено сообщение:', evt.data);
                    var e = document.getElementById('ledstatus');
                    if (evt.data === 'ledon') {
                        e.style.color = 'red';
                    } else if (evt.data === 'ledoff') {
                        e.style.color = 'black';
                    } else {
                        console.warn('Неизвестное событие:', evt.data);
                    }
                };
            } catch (error) {
                console.error('Ошибка при инициализации WebSocket:', error);
            }
        }

        function buttonclick(e) {
            try {
                if (websock && websock.readyState === WebSocket.OPEN) {
                    websock.send(e.id);
                    console.log('Отправлено сообщение:', e.id);
                } else {
                    console.error('WebSocket не открыт');
                }
            } catch (error) {
                console.error('Ошибка при отправке сообщения:', error);
            }
        }
    </script>
</head>
<body onload="start();">
    <table width="100%" border="1">
        <tr>
            <td bgcolor="#FFFF33" id="JD">Quadruped Controller</td>
        </tr>
    </table>
    <table width="100" height="249" border="0" align="center">
        <tr>
            <td align="center" valign="middle">
                <button id="w_20" type="button" onclick="buttonclick(this);" class="button red">Claw_Close</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_1_1" type="button" onclick="buttonclick(this);" class="button green">Forward</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_21" type="button" onclick="buttonclick(this);" class="button red">Claw_Open</button>
            </td>
        </tr>
        <tr>
            <td align="center" valign="middle">
                <button id="w_3_1" type="button" onclick="buttonclick(this);" class="button green">Turn_Left</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_0_1" type="button" onclick="buttonclick(this);" class="button red">Stop_all</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_4_1" type="button" onclick="buttonclick(this);" class="button green">Turn_Right</button>
            </td>
        </tr>
        <tr>
            <td>&nbsp;</td>
            <td align="center" valign="middle">
                <button id="w_2_1" type="button" onclick="buttonclick(this);" class="button green">Backward</button>
            </td>
            <td>&nbsp;</td>
        </tr>
        <tr>
            <td align="center" valign="middle">
                <button id="w_5_3" type="button" onclick="buttonclick(this);" class="button yellow">Shake</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_8_5" type="button" onclick="buttonclick(this);" class="button blue">Head_up</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_6_3" type="button" onclick="buttonclick(this);" class="button yellow">Wave</button>
            </td>
        </tr>
        <tr>
            <td align="center" valign="middle">
                <button id="w_16" type="button" onclick="buttonclick(this);" class="button blue">Twist_Left</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_9_5" type="button" onclick="buttonclick(this);" class="button blue">Head_down</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_17" type="button" onclick="buttonclick(this);" class="button blue">Twist_Right</button>
            </td>
        </tr>
        <tr>
            <td align="center" valign="middle">
                <button id="w_11_5" type="button" onclick="buttonclick(this);" class="button blue">Body_left</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_13" type="button" onclick="buttonclick(this);" class="button blue">Body_higher</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_10_5" type="button" onclick="buttonclick(this);" class="button blue">Body_right</button>
            </td>
        </tr>
        <tr>
            <td align="center" valign="middle">
                <button id="w_12" type="button" onclick="buttonclick(this);" class="button yellow">Service</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_14" type="button" onclick="buttonclick(this);" class="button blue">Body_lower</button>
            </td>
            <td align="center" valign="middle">
                <button id="w_15" type="button" onclick="buttonclick(this);" class="button yellow">Reset_Pose</button>
            </td>
        </tr>
        <tr>
            <td align="center" valign="middle">
                <button id="w_0_0" type="button" onclick="buttonclick(this);" class="button yellow">Walk</button>
            </td>
            <td align="center" valign="middle">&nbsp;</td>
            <td align="center" valign="middle">
                <button id="w_run" type="button" onclick="buttonclick(this);" class="button yellow">Run</button>
            </td>
        </tr>
    </table>
</body>
</html>
)=====";
