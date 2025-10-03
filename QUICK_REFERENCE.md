# 🕷️ Hexapod Quick Reference Card

## 🎮 Основные команды

### Движение
```
↑ FWD     ← LEFT      → RIGHT     ↓ BWD      ⏹ STOP
```

### Скорость
```
🐌 SLOW (200ms)    🚶 NORMAL (150ms)    🏃 FAST (120ms)
```

### Жесты
```
🤝 SHAKE    👋 WAVE
```

### Регулировки позы
```
Высота:      ⬆️ BODY_UP      ⬇️ BODY_DOWN
Голова:      🔼 HEAD_UP       🔽 HEAD_DOWN
Наклон:      ⬅️ LEAN_LEFT    ➡️ LEAN_RIGHT
Скручивание: ↶ TWIST_LEFT    ↷ TWIST_RIGHT
```

### Диагностика
```
CALIBRATE  DIAGNOSTIC  RESET  TRIPOD_TEST  JOINT_TEST  BATTERY_CHECK
```

### Аварийная остановка
```
⚠️ EMERGENCY STOP ⚠️
```

---

## 🔋 Индикатор батареи

| 🟢 > 11.3V | 🟡 10.5-11.3V | 🟠 9.5-10.5V | 🔴 < 9.5V STOP! |
|-----------|---------------|--------------|-----------------|

---

## 🦵 Схема ног

```
       FL(5)     FR(0)
        |   \   /   |
        |    \ /    |  
   ML(4)|     X     |MR(1)
        |    / \    |
        |   /   \   |
       RL(3)     RR(2)
```

**Тестирование:** `TEST_LEG_0` до `TEST_LEG_5`

---

## 🚀 Быстрый старт

1. `RESET` → проверка позы
2. `FWD` → движение вперёд
3. `LEFT/RIGHT` → повороты
4. `STOP` → остановка

---

## ⚠️ Важно!

- ✅ Регулировки позы **кумулятивные** (каждое нажатие добавляет +50/-50)
- ✅ **RESET** для сброса всех регулировок
- ❌ Не работайте при 🔴 < 9.5V
- ❌ Используйте **STOP** перед сменой направления

---

*Serial Monitor: 115200 baud | WebSocket: Port 81 | WiFi: Hexapod_Config (192.168.4.1)*

