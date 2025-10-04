const char PROGMEM PAGE_STYLES[] = R"=====(
/* ═══════════════════════════════════════════════════════════════
   HEXAPOD ROBOT WEB INTERFACE - STYLES
   ═══════════════════════════════════════════════════════════════ */

* {
    margin: 0;
    padding: 0;
    box-sizing: border-box;
}

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

/* Header */
.header {
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    padding: 20px;
    text-align: center;
    color: white;
}

.header h1 {
    font-size: 2em;
    margin-bottom: 10px;
}

/* Status Bar */
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

#connection-status {
    font-weight: bold;
    color: #dc3545;
}

#connection-status.connected {
    color: #28a745;
}

#command-status {
    font-weight: bold;
    color: #6c757d;
}

#command-status.executing {
    color: #ffc107;
}

#command-status.completed {
    color: #28a745;
}

/* Battery Indicator */
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

/* Tabs */
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

.tab-button:hover {
    background: rgba(255,255,255,0.5);
}

.tab-content {
    display: none;
    padding: 20px;
}

.tab-content.active {
    display: block;
}

/* Control Section */
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

/* Control Grid */
.control-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 10px;
    max-width: 400px;
    margin: 0 auto;
}

/* Test Grid */
.test-grid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 10px;
}

/* Buttons */
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

.stop {
    background: #dc3545;
    color: white;
}

.stop:hover {
    background: #c82333;
}

.test {
    background: #17a2b8;
    color: white;
}

.test:hover {
    background: #138496;
}

.diagnostic {
    background: #ffc107;
    color: #212529;
}

.diagnostic:hover {
    background: #e0a800;
}

/* Speed Controls */
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
}

.speed-switch {
    position: relative;
    display: inline-block;
    width: 60px;
    height: 34px;
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

input:checked + .speed-slider {
    background-color: #ff6b35;
}

input:checked + .speed-slider:before {
    transform: translateX(26px);
}

/* Hexapod Layout */
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

.leg-test:hover {
    background: #218838;
}

.hex-body {
    width: 100px;
    height: 100px;
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    border-radius: 15px;
    display: flex;
    align-items: center;
    justify-content: center;
}

.hex-symbol {
    font-size: 36px;
    color: white;
    font-weight: bold;
}

.hex-center-top, .hex-center-bottom {
    width: 100px;
    height: 20px;
}

/* Coordinates Display */
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

.coord-leg-name {
    font-weight: bold;
    margin-right: 10px;
}

.coord-left {
    color: #28a745;
}

.coord-right {
    color: #007bff;
}

/* Mobile Responsiveness */
@media (max-width: 768px) {
    body {
        padding: 5px;
    }
    
    .header h1 {
        font-size: 1.5em;
    }
    
    .status-bar {
        flex-direction: column;
        gap: 15px;
    }
    
    .control-section {
        padding: 15px;
        margin-bottom: 15px;
    }
    
    .control-grid {
        max-width: 100%;
    }
    
    .test-grid {
        grid-template-columns: repeat(2, 1fr);
    }
    
    button {
        padding: 12px 15px;
        font-size: 14px;
    }
    
    .leg-test {
        width: 60px;
        height: 60px;
        font-size: 12px;
    }
    
    .hex-body {
        width: 80px;
        height: 80px;
    }
    
    .hex-symbol {
        font-size: 24px;
    }
    
    .tab-button {
        padding: 12px 10px;
        font-size: 14px;
    }
}

@media (max-width: 480px) {
    .header h1 {
        font-size: 1.2em;
    }
    
    .test-grid {
        grid-template-columns: 1fr;
    }
    
    button {
        padding: 10px 12px;
        font-size: 13px;
    }
    
    .control-grid {
        gap: 8px;
    }
}
)=====";

